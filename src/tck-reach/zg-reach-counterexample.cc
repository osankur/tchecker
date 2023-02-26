#include <boost/dynamic_bitset.hpp>
#include <stack>

#include "tchecker/algorithms/search_order.hh"
#include "tchecker/ta/system.hh"
#include "tchecker/system/static_analysis.hh"
#include "tchecker/utils/log.hh"

#include "zg-reach-counterexample.hh"
namespace tchecker {

namespace tck_reach {

namespace zg_reach_counterexample {

#define DBM(i, j) dbm[(i)*dim + (j)]

/*!
 \brief Constrains the dbm to a single valuation that belongs to the given dbm, possibly after scaling it by factor.
 \param dbm : a dbm
 \param dim : dimension of dbm
 \param factor : scale factor; i.e. constant by which the dbm was scaled.
 \pre dbm is not nullptr (checked by assertion)
 dbm is a dim*dim array of difference bounds
 dbm is consistent (checked by assertion)
 dbm is tight (checked by assertion)
 dim >= 1 (checked by assertion).
 \post dbm was scaled by factor and represents a single valuation; factor was multiplied by the scaling factor
 \return new scale factor
 */
static tchecker::integer_t pick_valuation(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, const tchecker::integer_t factor)
{
  assert(!tchecker::dbm::is_empty_0(dbm, dim));
  assert(dbm != nullptr);
  assert(dim >= 1);
  assert(factor >= 1);
  for (tchecker::clock_id_t c = 1; c < dim; c++) {
    if (tchecker::dbm::comparator(DBM(0, c)) == tchecker::dbm::LE) {
      DBM(c, 0) = tchecker::dbm::db(tchecker::dbm::LE, -tchecker::dbm::value(DBM(0, c)));
      tchecker::dbm::tighten(dbm, dim);
    }
    else if (tchecker::dbm::comparator(DBM(c, 0)) == tchecker::dbm::LE) {
      DBM(0, c) = tchecker::dbm::db(tchecker::dbm::LE, -tchecker::dbm::value(DBM(c, 0)));
      tchecker::dbm::tighten(dbm, dim);
    }
    else if (DBM(c, 0) == tchecker::dbm::LT_INFINITY ||
             tchecker::dbm::value(DBM(c, 0)) > -tchecker::dbm::value(DBM(0, c)) + 1) {
      DBM(0, c) = tchecker::dbm::db(tchecker::dbm::LE, tchecker::dbm::value(DBM(0, c)) - 1);
      DBM(c, 0) = tchecker::dbm::db(tchecker::dbm::LE, -tchecker::dbm::value(DBM(0, c)));
      tchecker::dbm::tighten(dbm, dim);
    }
    else {
      for (tchecker::clock_id_t x = 0; x < dim; x++) {
        for (tchecker::clock_id_t y = 0; y < dim; y++) {
          if (DBM(x, y) != tchecker::dbm::LT_INFINITY) {
            auto new_value = tchecker::dbm::value(DBM(x, y)) * 2;
            DBM(x, y) = tchecker::dbm::db(tchecker::dbm::comparator(DBM(x, y)), new_value);
          }
        }
      }
      return pick_valuation(dbm, dim, 2*factor);
    }
  }
  assert(tchecker::dbm::is_consistent(dbm, dim));
  assert(tchecker::dbm::is_tight(dbm, dim));
  return factor;
}

/**
 * @brief Scale down all components of dbm by multiplying them by div when div is the largest number d that can be written as factor/2^i
 * for i in {0,1,2,...}, and which divides all components. 
 *
 * @param dbm : a dbm
 * @param dim : dimension of dbm
 * @param factor the initial scale factor
 * @return the new scale factor, that is, factor/d where d is the quantity by which all components were divided.
 */
static unsigned scale_down_dbm(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, const tchecker::integer_t factor)
{
  tchecker::integer_t div = factor;
  for (tchecker::clock_id_t x = 0; x < dim; x++) {
    for (tchecker::clock_id_t y = 0; y < dim; y++) {
      if (x == y)
        continue;
      if (dbm[x * dim + y] == tchecker::dbm::LT_INFINITY || tchecker::dbm::value(dbm[x * dim + y]) == 0) {
        continue;
      }
      auto v = tchecker::dbm::value(dbm[x * dim + y]);
      while (div > 1) {
        if ((v / div) * div == v) {
          break;
        }
        else {
          div /= 2;
        }
      }
    }
  }
  for (tchecker::clock_id_t x = 0; x < dim; x++) {
    for (tchecker::clock_id_t y = 0; y < dim; y++) {
      if (x == y)
        continue;
      if (dbm[x * dim + y] == tchecker::dbm::LT_INFINITY || tchecker::dbm::value(dbm[x * dim + y]) == 0) {
        continue;
      }
      dbm[x * dim + y] =
          tchecker::dbm::db(tchecker::dbm::comparator(dbm[x * dim + y]), tchecker::dbm::value(dbm[x * dim + y]) / div);
    }
  }
  return factor / div;
}

/**
 * @brief Multiply all constants of the dbm by factor
 *
 * @param dbm : a dbm
 * @param dim : dimension of dbm
 * @param factor target scale factor
 */
static void scale_dbm(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, const tchecker::integer_t factor)
{
  for (tchecker::clock_id_t x = 0; x < dim; x++) {
    for (tchecker::clock_id_t y = 0; y < dim; y++) {
      if (x == y)
        continue;
      if (dbm[x * dim + y] == tchecker::dbm::LT_INFINITY) {
        continue;
      }
      auto new_value = tchecker::dbm::value(dbm[x * dim + y]) * factor;
      dbm[x * dim + y] = tchecker::dbm::db(tchecker::dbm::comparator(dbm[x * dim + y]), new_value);
    }
  }
}

/**
 * @brief String of clock valuation
 * 
 * @param val 
 * @param clock_name 
 * @return std::string 
 */
static std::string string_of_valuation(std::vector<double> & val, std::function<std::string(tchecker::clock_id_t)> clock_name) {
    std::stringstream ss;
    for(tchecker::clock_id_t i = 1; i < val.size(); i++){
        ss << clock_name(i) << "=" << val[i];
        if(i < val.size()-1){
            ss << ", ";
        }
    }
    return ss.str();
}

/**
 * @brief Find a path from an initial state to an accepting state in the graph, and commpute a concrete execution along this.
 * @pre Graph g contains a path from an initial state to an accepting state
 * @param g 
 * @param edges
 * @param states
 * @return counterexample 
 */

std::shared_ptr<tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t>
generate_concrete_trace(std::shared_ptr<tchecker::ta::system_t const> system, tchecker::tck_reach::zg_reach_counterexample::graph_t const & g,
                        tchecker::zg::semantics_type_t semantics_type)
{
  using node_sptr_t = tchecker::tck_reach::zg_reach_counterexample::graph_t::node_sptr_t;
  using edge_sptr_t = tchecker::tck_reach::zg_reach_counterexample::graph_t::edge_sptr_t;
  auto cex = std::make_shared<tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t>();
  node_sptr_t currentNode;
  std::map<std::string, std::string> state_attr;
  std::map<std::string, std::string> edge_attr;
  std::vector<double> val;
  std::vector<double> val_reset;
  std::vector<tchecker::dbm::db_t *> Z;
  std::stringstream ss;
  std::list<tchecker::tck_reach::zg_reach_counterexample::graph_t::node_sptr_t> states;
  std::list<tchecker::tck_reach::zg_reach_counterexample::graph_t::edge_sptr_t> edges;

  // Find path to accepting state inside the zone graph
  for (node_sptr_t const & n : g.nodes()) {
    if (n->is_accepting()) {
      currentNode = n;
      break;
    }
  }
  if (!currentNode->is_accepting()) {
    throw std::runtime_error("No accepting node in the zone graph\n");
  }

  state_attr.clear();
  g.attributes(currentNode, state_attr);
  cex->state_attributes.push_back(state_attr);

  states.push_front(currentNode);
  while (!currentNode->is_initial()) {
    for (edge_sptr_t const & e : g.incoming_edges(currentNode)) {
      edge_attr.clear();
      g.attributes(e, edge_attr);

      if (edge_attr["parent"] == "true") {
        currentNode = g.edge_src(e);
        states.push_front(currentNode);
        edges.push_front(e);
        state_attr.clear();
        g.attributes(currentNode, state_attr);
        cex->state_attributes.push_back(state_attr);
        cex->edge_attributes.push_back(edge_attr);
        break;
      }
    }
  }
  std::reverse(cex->state_attributes.begin(), cex->state_attributes.end());
  std::reverse(cex->edge_attributes.begin(), cex->edge_attributes.end());
  // Compute zones forwards Z[0], Z[1], Z[2], ... without extrapolation
  auto semantics = tchecker::zg::semantics_factory(semantics_type);
  tchecker::clock_id_t dim = currentNode->state().zone().dim();

  // Initial zone Z[0] = the zone of states[0] intersected with diagonal of vec{0}
  tchecker::dbm::db_t * init = new tchecker::dbm::db_t[dim * dim];
  tchecker::dbm::zero(init, dim);
  tchecker::dbm::open_up(init, dim);
  tchecker::dbm::db_t * d = new tchecker::dbm::db_t[dim * dim];
  for (tchecker::clock_id_t x = 0; x < dim * dim; x++) {
    d[x] = currentNode->state().zone().dbm()[x];
  }
  tchecker::dbm::intersection(d, d, init, dim);

  tchecker::dbm::db_t * z = nullptr;
  z = new tchecker::dbm::db_t[dim * dim];
  std::memcpy(z, d, dim * dim * sizeof(tchecker::dbm::db_t));
  Z.push_back(z);
  ss.str("");
  tchecker::dbm::output(ss, z, dim,
        [&](tchecker::clock_id_t id) { return (id == 0 ? "0" : system->clock_variables().flattened().index().value(id - 1)); });
  cex->state_attributes.at(0)["concrete_zone"] = ss.str();

  auto edge_it = edges.begin();
  auto state_it = states.begin();
  int i = 0;
  while (edge_it != edges.end()) {
    auto & e = *(*edge_it);
    bool src_delay_allowed = tchecker::ta::delay_allowed(*system, *(*state_it)->state_ptr()->vloc_ptr());
    bool tgt_delay_allowed = tchecker::ta::delay_allowed(*system, *(*std::next(state_it))->state_ptr()->vloc_ptr());
    semantics->next(d, dim, src_delay_allowed, e.src_invariant_container(), e.guard_container(), e.reset_container(),
                    tgt_delay_allowed, e.tgt_invariant_container());

    edge_it++;
    state_it++;
    i++;

    z = new tchecker::dbm::db_t[dim * dim];
    std::memcpy(z, d, dim * dim * sizeof(tchecker::dbm::db_t));
    Z.push_back(z);
    ss.str("");
    tchecker::dbm::output(ss, z, dim,
        [&](tchecker::clock_id_t id) { return (id == 0 ? "0" : system->clock_variables().flattened().index().value(id - 1)); });
    cex->state_attributes.at(i)["concrete_zone"] = ss.str();
  }

  // Pick valuations by traversing this path bacwards
  // At each step: Constrain the current dbm to a single valuation,
  //    compute predecessor, intersect with Z[i]
  try {
    tchecker::integer_t factor = 1;
    factor = pick_valuation(d, dim, factor);

    val.clear();
    for (tchecker::clock_id_t c = 0; c < dim; c++) {
      val.push_back(tchecker::dbm::value(d[c * dim]) / (double)factor);
    }

    auto edge_rit = edges.rbegin();
    auto state_rit = states.rbegin();
    auto z_rit = ++Z.rbegin();
    int i = Z.size() - 2;
    while (edge_rit != edges.rend()) {
      auto & e = *(*edge_rit);
      bool tgt_delay_allowed = tchecker::ta::delay_allowed(*system, *(*state_rit)->state_ptr()->vloc_ptr());

      tchecker::clock_constraint_container_t tgt_invariant = e.tgt_invariant_container();
      tchecker::clock_constraint_container_t src_invariant = e.src_invariant_container();
      tchecker::clock_constraint_container_t guard = e.guard_container();

      // Scale the guards and invariants
      for (auto & c : tgt_invariant) {
        c.value() *= factor;
      }
      for (auto & c : src_invariant) {
        c.value() *= factor;
      }
      for (auto & c : guard) {
        c.value() *= factor;
      }

      tchecker::dbm::constrain(d, dim, tgt_invariant);
      if (tgt_delay_allowed) {
        tchecker::dbm::open_down(d, dim);
      }

      for (auto c : e.reset_container()) {
        if (c.left_id() != tchecker::REFCLOCK_ID) {
          tchecker::dbm::free(d, dim, c.left_id() + 1);
        }
      }

      tchecker::dbm::constrain(d, dim, guard);
      tchecker::dbm::constrain(d, dim, src_invariant);

      scale_dbm(*z_rit, dim, factor);
      tchecker::dbm::intersection(d, d, *z_rit, dim);

      factor = scale_down_dbm(d, dim, factor);
      factor = pick_valuation(d, dim, factor);

      val.clear();
      val_reset.clear();
      for (tchecker::clock_id_t c = 0; c < dim; c++) {
        val.push_back(tchecker::dbm::value(d[c * dim]) / (double)factor);
        val_reset.push_back(tchecker::dbm::value(d[c * dim]) / (double)factor);
      }

      for (auto r : e.reset_container()) {
        tchecker::clock_id_t lid = (r.left_id() == tchecker::REFCLOCK_ID ? 0 : r.left_id() + 1);
        tchecker::clock_id_t rid = (r.right_id() == tchecker::REFCLOCK_ID ? 0 : r.right_id() + 1);
        if (lid != 0) {
          if (rid == 0) {
            val_reset.at(lid) = r.value();
          }
          else {
            val_reset.at(lid) = (val_reset.at(rid) + r.value());
          }
        }
      }
      cex->state_attributes[i+1]["val1"] = string_of_valuation(val_reset, [&](tchecker::clock_id_t id) { return (id == 0 ? "0" : system->clock_variables().flattened().index().value(id - 1)); });
      cex->state_attributes[i]["val2"] = string_of_valuation(val, [&](tchecker::clock_id_t id) { return (id == 0 ? "0" : system->clock_variables().flattened().index().value(id - 1)); });

      edge_rit++;
      state_rit++;
      z_rit++;
      i--;
    }
    // Add initial valuation
    val.clear();
    for (tchecker::clock_id_t c = 0; c < dim; c++) {
      val.push_back(0);
    }
    cex->state_attributes[0]["val1"] = string_of_valuation(val, [&](tchecker::clock_id_t id) { return (id == 0 ? "0" : system->clock_variables().flattened().index().value(id - 1)); });
  }
  catch (tchecker::dbm::overflow & ex) {
    std::cerr << tchecker::log_warning << "Cannot compute concrete trace: " << ex.what() << "\n";
  }

  for (auto z : Z) {
    delete[] z;
  }
  delete[] d;
  delete[] init;
  delete semantics;

  return cex;
}


/* node_t */

node_t::node_t(tchecker::zg::state_sptr_t const & s) : _accepting(false),  _init(false), _state(s) {}

node_t::node_t(tchecker::zg::const_state_sptr_t const & s) : _accepting(false), _init(false), _state(s) {}

/* node_hash_t */

std::size_t node_hash_t::operator()(tchecker::tck_reach::zg_reach_counterexample::node_t const & n) const { return hash_value(n.state()); }

/* node_equal_to_t */

bool node_equal_to_t::operator()(tchecker::tck_reach::zg_reach_counterexample::node_t const & n1,
                                 tchecker::tck_reach::zg_reach_counterexample::node_t const & n2) const
{
  return n1.state() == n2.state();
}

/* edge_t */
edge_t::edge_t(tchecker::zg::transition_t const & t) :_src_invariant(t.src_invariant_container()), 
                                                      _guard(t.guard_container()),
                                                      _reset(t.reset_container()),                                                      
                                                      _tgt_invariant(t.tgt_invariant_container()), 
                                                      _vedge(t.vedge_ptr()) {}

/* edge_t */
edge_t::edge_t(tchecker::zg::shared_transition_t const & t) :_src_invariant(t.src_invariant_container()), 
                                                      _guard(t.guard_container()),
                                                      _reset(t.reset_container()),                                                      
                                                      _tgt_invariant(t.tgt_invariant_container()), 
                                                      _vedge(t.vedge_ptr()) {}


/* graph_t */

graph_t::graph_t(std::shared_ptr<tchecker::zg::sharing_zg_t> const & zg, std::size_t block_size, std::size_t table_size)
    : tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach_counterexample::node_t, tchecker::tck_reach::zg_reach_counterexample::edge_t,
                                             tchecker::tck_reach::zg_reach_counterexample::node_hash_t,
                                             tchecker::tck_reach::zg_reach_counterexample::node_equal_to_t>(
            block_size, table_size, tchecker::tck_reach::zg_reach_counterexample::node_hash_t(),
            tchecker::tck_reach::zg_reach_counterexample::node_equal_to_t()),
      _zg(zg)
{
}


graph_t::~graph_t()
{
  tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach_counterexample::node_t, tchecker::tck_reach::zg_reach_counterexample::edge_t,
                                         tchecker::tck_reach::zg_reach_counterexample::node_hash_t,
                                         tchecker::tck_reach::zg_reach_counterexample::node_equal_to_t>::clear();
}

void graph_t::attributes(tchecker::tck_reach::zg_reach_counterexample::node_t const & n, std::map<std::string, std::string> & m) const
{
  _zg->attributes(n.state_ptr(), m);
}

void graph_t::attributes(tchecker::tck_reach::zg_reach_counterexample::edge_t const & e, std::map<std::string, std::string> & m) const
{
  m["vedge"] = tchecker::to_string(e.vedge(), _zg->system().as_system_system());
}

/* dot_output */

/*!
 \class node_lexical_less_t
 \brief Less-than order on nodes based on lexical ordering
*/
class node_lexical_less_t {
public:
  /*!
   \brief Less-than order on nodes based on lexical ordering
   \param n1 : a node
   \param n2 : a node
   \return true if n1 is less-than n2 w.r.t. lexical ordering over the states in
   the nodes
  */
  bool operator()(tchecker::tck_reach::zg_reach_counterexample::graph_t::node_sptr_t const & n1,
                  tchecker::tck_reach::zg_reach_counterexample::graph_t::node_sptr_t const & n2) const
  {
    return tchecker::zg::lexical_cmp(n1->state(), n2->state()) < 0;
  }
};

/*!
 \class edge_lexical_less_t
 \brief Less-than ordering on edges based on lexical ordering
 */
class edge_lexical_less_t {
public:
  /*!
   \brief Less-than ordering on edges based on lexical ordering
   \param e1 : an edge
   \param e2 : an edge
   \return true if e1 is less-than  e2 w.r.t. the tuple of edges in e1 and e2
  */
  bool operator()(tchecker::tck_reach::zg_reach_counterexample::graph_t::edge_sptr_t const & e1,
                  tchecker::tck_reach::zg_reach_counterexample::graph_t::edge_sptr_t const & e2) const
  {
    return tchecker::lexical_cmp(e1->vedge(), e2->vedge()) < 0;
  }
};

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_reach_counterexample::graph_t const & g, std::string const & name)
{
  return tchecker::graph::reachability::dot_output<tchecker::tck_reach::zg_reach_counterexample::graph_t,
                                                   tchecker::tck_reach::zg_reach_counterexample::node_lexical_less_t,
                                                   tchecker::tck_reach::zg_reach_counterexample::edge_lexical_less_t>(os, g, name);
}

std::ostream & dot_cex_output(std::ostream & os, tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t const & cex, std::string const & name){
  tchecker::graph::dot_output_header(os, name);

  int i = 0;
  for(auto attr : cex.state_attributes){
    tchecker::graph::dot_output_node(os, std::to_string(i), attr);
    i++;
  }

  i = 0;  
  for(auto attr : cex.edge_attributes){
    tchecker::graph::dot_output_edge(os, std::to_string(i), std::to_string(i+1), attr);
    i++;
  }
  tchecker::graph::dot_output_footer(os);
  return os;
}


/* run */

std::tuple<tchecker::algorithms::reach::stats_t, std::shared_ptr<tchecker::tck_reach::zg_reach_counterexample::graph_t>, std::shared_ptr<tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t>>
run(std::shared_ptr<tchecker::parsing::system_declaration_t> const & sysdecl, std::string const & labels,
    std::string const & search_order, std::size_t block_size, std::size_t table_size)
{
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{*sysdecl}};
  if (!tchecker::system::every_process_has_initial_location(system->as_system_system()))
    std::cerr << tchecker::log_warning << "system has no initial state" << std::endl;

  std::shared_ptr<tchecker::zg::sharing_zg_t> zg{tchecker::zg::factory_sharing(
      system, tchecker::zg::ELAPSED_SEMANTICS, tchecker::zg::EXTRA_LU_PLUS_LOCAL, block_size, table_size)};

  std::shared_ptr<tchecker::tck_reach::zg_reach_counterexample::graph_t> graph{
      new tchecker::tck_reach::zg_reach_counterexample::graph_t{zg, block_size, table_size}};

  boost::dynamic_bitset<> accepting_labels = system->as_syncprod_system().labels(labels);

  tchecker::tck_reach::zg_reach_counterexample::algorithm_t algorithm;

  enum tchecker::waiting::policy_t policy = tchecker::algorithms::waiting_policy(search_order);

  tchecker::algorithms::reach::stats_t stats = algorithm.run(*zg, *graph, accepting_labels, policy);
  std::shared_ptr<tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t> cex = 
    std::make_shared<tchecker::tck_reach::zg_reach_counterexample::counterexample_trace_t>();  
  if(stats.reachable()){
    cex = generate_concrete_trace(system, *graph, tchecker::zg::ELAPSED_SEMANTICS);
  }

  return std::make_tuple(stats, graph, cex);
}


} // namespace zg_reach_counterexample
} // namespace tck_reach
} // namespace tchecker
