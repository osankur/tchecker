/*
 * This file is a part of the TChecker project.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include <boost/dynamic_bitset.hpp>
#include <stack>

#include "tchecker/algorithms/search_order.hh"
#include "tchecker/ta/system.hh"
#include "zg-reach.hh"

namespace tchecker {

namespace tck_reach {

namespace zg_reach {

/* node_t */

node_t::node_t(tchecker::zg::state_sptr_t const & s) : _unsafe(false),  _init(false), _state(s) {}

node_t::node_t(tchecker::zg::const_state_sptr_t const & s) : _unsafe(false), _init(false), _state(s) {}

/* node_hash_t */

std::size_t node_hash_t::operator()(tchecker::tck_reach::zg_reach::node_t const & n) const { return hash_value(n.state()); }

/* node_equal_to_t */

bool node_equal_to_t::operator()(tchecker::tck_reach::zg_reach::node_t const & n1,
                                 tchecker::tck_reach::zg_reach::node_t const & n2) const
{
  return n1.state() == n2.state();
}

/* edge_t */

edge_t::edge_t(tchecker::zg::transition_t const & t) : _vedge(t.vedge_ptr()) {}

/* graph_t */

graph_t::graph_t(std::shared_ptr<tchecker::zg::zg_t> const & zg, std::size_t block_size, std::size_t table_size)
    : tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach::node_t, tchecker::tck_reach::zg_reach::edge_t,
                                             tchecker::tck_reach::zg_reach::node_hash_t,
                                             tchecker::tck_reach::zg_reach::node_equal_to_t>(block_size, table_size),
      _zg(zg)
{
}

graph_t::~graph_t()
{
  tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach::node_t, tchecker::tck_reach::zg_reach::edge_t,
                                         tchecker::tck_reach::zg_reach::node_hash_t,
                                         tchecker::tck_reach::zg_reach::node_equal_to_t>::clear();
}

void graph_t::attributes(tchecker::tck_reach::zg_reach::node_t const & n, std::map<std::string, std::string> & m) const
{
  _zg->attributes(n.state_ptr(), m);
}

void graph_t::attributes(tchecker::tck_reach::zg_reach::edge_t const & e, std::map<std::string, std::string> & m) const
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
  bool operator()(tchecker::tck_reach::zg_reach::graph_t::node_sptr_t const & n1,
                  tchecker::tck_reach::zg_reach::graph_t::node_sptr_t const & n2) const
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
  bool operator()(tchecker::tck_reach::zg_reach::graph_t::edge_sptr_t const & e1,
                  tchecker::tck_reach::zg_reach::graph_t::edge_sptr_t const & e2) const
  {
    return tchecker::lexical_cmp(e1->vedge(), e2->vedge()) < 0;
  }
};

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_reach::graph_t const & g, std::string const & name)
{
  return tchecker::graph::reachability::dot_output<tchecker::tck_reach::zg_reach::graph_t,
                                                   tchecker::tck_reach::zg_reach::node_lexical_less_t,
                                                   tchecker::tck_reach::zg_reach::edge_lexical_less_t>(os, g, name);
}

std::ostream & dot_cex_output(std::ostream & os, tchecker::tck_reach::zg_reach::graph_t const & g, std::string const & name){
  return tchecker::graph::reachability::dot_cex_output<tchecker::tck_reach::zg_reach::graph_t>(os, g, name);
}



/**
 * @brief Find a path from an initial state to an unsafe state and add concrete valuations that form a run as attributes
 *        states[i] ---trace[i]---> states[i+1]
 * @pre Graph g contains a path from an initial state to an unsafe state
 * @pre edges and states point to empty lists
 * @param g 
 * @param edges
 * @param states
 * @post The states traversed by an error trace have been written to states;
 *  the edges used written to edges, and a sequence of clock valuations to clock_valuations.
 */
void generate_concrete_trace(tchecker::tck_reach::zg_reach::graph_t const & g,
      tchecker::zg::zg_t const & zg,
      std::shared_ptr<std::list<tchecker::tck_reach::zg_reach::graph_t::node_sptr_t>> & states,
      std::shared_ptr<std::list<tchecker::tck_reach::zg_reach::graph_t::edge_sptr_t>> & edges,
      std::shared_ptr<std::list<std::vector<double>>> & clock_valuations
    ){
  using node_sptr_t = tchecker::tck_reach::zg_reach::graph_t::node_sptr_t;
  using edge_sptr_t = tchecker::tck_reach::zg_reach::graph_t::edge_sptr_t;
  assert(edges->empty());
  assert(states->empty());
  assert(clock_valuations->empty());
  node_sptr_t currentNode;
  std::map<std::string, std::string> attr;
  std::vector<double> val;

  for (node_sptr_t const & n : g.nodes()){
    if (n->is_unsafe()){
      currentNode = n;
      break;
    }
  }
  if (!currentNode->is_unsafe()){
    throw std::runtime_error("No unsafe node in the zone graph\n");
  }

  tchecker::clock_id_t dim = currentNode->state().zone().dim();
  tchecker::dbm::db_t * d = new tchecker::dbm::db_t[dim*dim];
  int factor = 1;

  currentNode->state().zone().to_dbm(d);
  tchecker::dbm::pick_valuation(d, dim, factor);
  val.clear();
  for(tchecker::clock_id_t c = 0; c < dim; c++){
    val.push_back(tchecker::dbm::value(d[c]) / (double) factor);
  }
  clock_valuations->push_front(val);

  while(!currentNode->is_initial()){
    
    // attr.clear();
    // g.attributes(currentNode, attr);
    // tchecker::graph::dot_output_node(os, "", attr);

    for (edge_sptr_t const & e : g.incoming_edges(currentNode)) {
      // std::cout << "Trying the following edge:\n";
      attr.clear();
      g.attributes(e, attr);
      // tchecker::graph::dot_output_edge(os, "", "", attr);

      if (attr["parent"] == "true"){
        currentNode = g.edge_src(e);
        states->push_front(currentNode);
        edges->push_front(e); 

        // TODO Create a DBM out of the integer valuation (obtained by * factor)
        // TODO Compute predecessor along the parent edge
        // TODO pick_valuation again ....
        // Add this valuation as an attribute
        break;
      }
    }
  }
  delete [] d;
}

/* run */

std::tuple<tchecker::algorithms::reach::stats_t, std::shared_ptr<tchecker::tck_reach::zg_reach::graph_t>>
run(std::shared_ptr<tchecker::parsing::system_declaration_t> const & sysdecl, std::string const & labels,
    std::string const & search_order, std::size_t block_size, std::size_t table_size)
{
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{*sysdecl}};

  std::shared_ptr<tchecker::zg::zg_t> zg{
      tchecker::zg::factory(system, tchecker::zg::ELAPSED_SEMANTICS, tchecker::zg::EXTRA_LU_PLUS_LOCAL, block_size)};

  std::shared_ptr<tchecker::tck_reach::zg_reach::graph_t> graph{
      new tchecker::tck_reach::zg_reach::graph_t{zg, block_size, table_size}};

  boost::dynamic_bitset<> accepting_labels = system->as_syncprod_system().labels(labels);

  tchecker::tck_reach::zg_reach::algorithm_t algorithm;

  enum tchecker::waiting::policy_t policy = tchecker::algorithms::waiting_policy(search_order);

  tchecker::algorithms::reach::stats_t stats = algorithm.run(*zg, *graph, accepting_labels, policy);
  if(stats.reachable()){
    auto edges = std::make_shared<std::list<tchecker::tck_reach::zg_reach::graph_t::edge_sptr_t>>();
    auto states = std::make_shared<std::list<tchecker::tck_reach::zg_reach::graph_t::node_sptr_t>>();
    auto clock_valuations = std::make_shared<std::list<std::vector<double>>>();
    generate_concrete_trace(*graph, *zg, states, edges, clock_valuations);
  }
  return std::make_tuple(stats, graph);
}

} // end of namespace zg_reach

} // end of namespace tck_reach

} // end of namespace tchecker