#ifndef TCHECKER_TCK_INTERPOLATE_INTERPOLATE_HH
#define TCHECKER_TCK_INTERPOLATE_INTERPOLATE_HH

/*!
 \file graph.hh
 \brief Simulation graph
*/

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cstdlib>
#include <map>
#include <memory>
#include <ostream>
#include <string>

#include "../tck-simulate/simulate.hh"
#include "graph.hh"
#include "tchecker/graph/reachability_graph.hh"
#include "tchecker/syncprod/vedge.hh"
#include "tchecker/utils/shared_objects.hh"
#include "tchecker/zg/zg.hh"
namespace tchecker {
namespace tck_interpolate {

class zg_t : public tchecker::zg::zg_t {
public:
  zg_t(std::shared_ptr<tchecker::ta::system_t const> const & system, std::unique_ptr<tchecker::zg::semantics_t> && semantics,
       std::unique_ptr<tchecker::zg::extrapolation_t> && extrapolation, std::size_t block_size)
      : tchecker::zg::zg_t(system, std::move(semantics), std::move(extrapolation), block_size)
  {
  }
  virtual void next(tchecker::zg::const_state_sptr_t const & s, std::vector<sst_t> & v)
  {
    tchecker::ts::full_ts_t<tchecker::zg::state_sptr_t, tchecker::zg::const_state_sptr_t, tchecker::zg::transition_sptr_t,
                            tchecker::zg::const_transition_sptr_t, tchecker::zg::initial_range_t,
                            tchecker::zg::outgoing_edges_range_t, tchecker::zg::initial_value_t,
                            tchecker::zg::outgoing_edges_value_t>::next(s, v,
                                                                        tchecker::STATE_OK |
                                                                            tchecker::STATE_CLOCKS_GUARD_VIOLATED |
                                                                            tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED |
                                                                            tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED |
                                                                            tchecker::STATE_ZONE_EMPTY);
  }
};

tchecker::tck_interpolate::zg_t * factory(std::shared_ptr<tchecker::ta::system_t const> const & system,
                                          enum tchecker::zg::semantics_type_t semantics_type,
                                          enum tchecker::zg::extrapolation_type_t extrapolation_type, std::size_t block_size)
{
  std::unique_ptr<tchecker::zg::extrapolation_t> extrapolation{
      tchecker::zg::extrapolation_factory(extrapolation_type, *system)};
  if (extrapolation.get() == nullptr)
    return nullptr;
  std::unique_ptr<tchecker::zg::semantics_t> semantics{tchecker::zg::semantics_factory(semantics_type)};
  return new tchecker::tck_interpolate::zg_t(system, std::move(semantics), std::move(extrapolation), block_size);
}

/*!
 \brief Random selection
 \param v : a vector of triples (status, state, transition)
 \pre the size of v is less than NO_SELECTION (checked by assertion)
 \return the index of the chosen element in v if v is not empty,
 tchecker::tck_simulate::NO_SELECTION otherwise
*/
static std::size_t randomized_select(std::vector<tchecker::tck_interpolate::zg_t::sst_t> const & v)
{
  if (v.size() == 0)
    return 1000;
  return std::rand() % v.size();
}
/*!
 \brief Display state
 \param os : output stream
 \param zg : zone graph
 \param s : state
 \post Attributes of state s have been displayed on os
 */
static void display(std::ostream & os, tchecker::zg::zg_t const & zg, tchecker::zg::const_state_sptr_t const & s)
{
  std::map<std::string, std::string> attr;
  zg.attributes(s, attr);
  for (auto && [key, value] : attr)
    os << "\t" << key << ": " << value << std::endl;
}

/*!
 \brief Display transition
 \param os : output stream
 \param zg : zone graph
 \param t : transition
 \post Attributes of transition t have been displayed on os
 */
static void display(std::ostream & os, tchecker::zg::zg_t const & zg, tchecker::zg::const_transition_sptr_t const & t)
{
  std::map<std::string, std::string> attr;
  zg.attributes(t, attr);
  for (auto && [key, value] : attr)
    os << "\t" << key << ": " << value << " ";
}

std::shared_ptr<tchecker::tck_interpolate::graph_t>
randomized_simulation(tchecker::parsing::system_declaration_t const & sysdecl, std::size_t nsteps)
{
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{sysdecl}};
  std::shared_ptr<tchecker::tck_interpolate::zg_t> zg{
      tchecker::tck_interpolate::factory(system, tchecker::zg::STANDARD_SEMANTICS, tchecker::zg::NO_EXTRAPOLATION, 100)};
  std::shared_ptr<tchecker::tck_interpolate::graph_t> g{new tchecker::tck_interpolate::graph_t{zg, 100}};
  std::vector<tchecker::tck_interpolate::zg_t::sst_t> v;
  auto const & z = *zg;

  srand(time(NULL));

  zg->initial(v);
  std::size_t k = tchecker::tck_interpolate::randomized_select(v);
  if (k == 1000)
    return g;
  tchecker::tck_interpolate::graph_t::node_sptr_t previous_node = g->add_node(zg->state(v[k]));
  display(std::cout, z, previous_node->state_ptr());
  std::cout << "\n\n";
  v.clear();

  for (std::size_t i = 0; i < nsteps; ++i) {
    zg->next(previous_node->state_ptr(), v);

    std::size_t k = tchecker::tck_interpolate::randomized_select(v);
    if (k == 1000)
      break;
    tchecker::tck_interpolate::graph_t::node_sptr_t node = g->add_node(zg->state(v[k]));

    tchecker::zg::const_transition_sptr_t trans{zg->transition(v[k])};
    display(std::cout, z, trans);
    std::cout << "\n";
    display(std::cout, z, node->state_ptr());
    if (node->state().zone().is_empty()) {
      std::cout << "\nempty";
    }
    std::cout << "\n\n";

    g->add_edge(previous_node, node, tchecker::graph::reachability::edge_type_t::EDGE_PARENT, zg->transition(v[k]));
    v.clear();

    previous_node = node;
  }

  return g;
}
/**
 * @brief Given a vedge attribute of the form <A@a,B@b,C@c,...> check if sigma appears 
 * among {a,b,c,...}.
 * 
 * @param vedge the vedge attribute of a transition
 * @param sigma an event
 * @return true if the vedge contains an edge on event sigma
 * @return false otherwise
 */
bool vedge_matches_sigma(std::string vedge, std::string & sigma){
  boost::algorithm::trim(vedge);
  vedge = vedge.substr(1,vedge.size()-2); // remove < and >
  std::vector<std::string> labels;
  boost::algorithm::split(labels, vedge, boost::is_any_of(","));
  // std::cout << "Matching `" << vedge << "' for sigma=" << sigma << "\n";
  for (auto & l : labels){
    std::vector<std::string> pair; // pair of process name and edge label
    boost::algorithm::split(pair, l, boost::is_any_of("@"));
    if (pair.size() != 2){
      throw std::runtime_error("Edge sync label could not be parsed:" + l);
    }
    // std::cout << "Got proc: <" << pair[0] << "> and <" << pair[1] << "> : match = " << (pair[1] == sigma) << "\n";
    if (pair[1] == sigma){
      return true;
    }
  }
  return false;
}

void compute_interpolant_automaton(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl, std::string trace,
                                   std::ostream & os)
{
  // tchecker::tck_interpolate::randomized_simulation(*sysdecl, 10);
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{*sysdecl}};
  std::shared_ptr<tchecker::tck_interpolate::zg_t> zg{
      tchecker::tck_interpolate::factory(system, tchecker::zg::STANDARD_SEMANTICS, tchecker::zg::NO_EXTRAPOLATION, 100)};
  // std::shared_ptr<tchecker::tck_interpolate::graph_t> g{new tchecker::tck_interpolate::graph_t{zg, 100}};
  std::vector<tchecker::tck_interpolate::zg_t::sst_t> v;

  std::vector<std::string> vtrace;
  boost::algorithm::split(vtrace, trace, boost::is_any_of(" \t"), boost::token_compress_on);
  for (auto t : vtrace) {
    std::cout << t << std::endl;
  }
  std::vector<tchecker::zg::const_state_sptr_t> states;
  std::vector<tchecker::zg::const_transition_sptr_t> trans;

  zg->initial(v);
  if (v.size() == 0) {
    throw std::runtime_error("Empty initial state set");
  }
  else if (v.size() > 1) {
    throw std::runtime_error("More than one initial state");
  }
  tchecker::zg::const_state_sptr_t curstate(zg->state(v[0]));
  states.push_back(curstate);
  for (auto sigma : vtrace) {
    v.clear();
    zg->next(curstate, v);
    tchecker::zg::const_transition_sptr_t nextt;
    for(unsigned i = 0; i < v.size(); i++){
        tchecker::zg::const_transition_sptr_t t{zg->transition(v[i])};
        std::map<std::string, std::string> attr;
        zg->attributes(t, attr);
        for (auto && [key, value] : attr){
          if (key == "vedge")
            if (vedge_matches_sigma(value, sigma)){
              if (nextt.ptr() != nullptr){
                throw std::runtime_error("The TA must be deterministic. There are multiple transitions for label " + sigma);
              } 
              nextt = t;
              trans.push_back(nextt);
              curstate = tchecker::zg::const_state_sptr_t(zg->state(v[i]));
              states.push_back(curstate);
            }
        }
    } 
    // The FSM does not allow this sigma. Stop simulation here.
    if (nextt.ptr() == nullptr){
      break;
    }
  }

  // Display
  display(std::cout, *zg, states[0]);
  std::cout << "\n\n";
  for(unsigned i = 0; i < trans.size(); i++){
    display(std::cout, *zg, trans[i]);
    std::cout << "\n";
    display(std::cout, *zg, states[i+1]);
    std::cout << "\n\n";
  }
}

} // namespace tck_interpolate
} // namespace tchecker
#endif