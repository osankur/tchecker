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
#include <unordered_map>
#include <memory>
#include <ostream>
#include <string>
#include <functional>

#include "../tck-simulate/simulate.hh"
// #include "graph.hh"
#include "tchecker/graph/reachability_graph.hh"
#include "tchecker/syncprod/vedge.hh"
#include "tchecker/utils/shared_objects.hh"
#include "tchecker/zg/zg.hh"
#include "tchecker/zg/allocators.hh"
#include "tchecker/zg/state.hh"
#include "tchecker/zg/transition.hh"
#include "tchecker/ts/ts.hh"

namespace tchecker {
namespace tck_tar {

/**
 * @brief zone graph that generates successors regardless whether the target zones are empty or not
 * 
 */
class zg_t : public tchecker::zg::zg_t {
public:
  zg_t(std::shared_ptr<tchecker::ta::system_t const> const & system, std::unique_ptr<tchecker::zg::semantics_t> && semantics,
       std::unique_ptr<tchecker::zg::extrapolation_t> && extrapolation, std::size_t block_size)
      : tchecker::zg::zg_t(system, std::move(semantics), std::move(extrapolation), block_size)
  {
  }

  using sst_t = tchecker::ts::full_ts_t<tchecker::zg::state_sptr_t, tchecker::zg::const_state_sptr_t, tchecker::zg::transition_sptr_t,
                            tchecker::zg::const_transition_sptr_t, tchecker::zg::initial_range_t,
                            tchecker::zg::outgoing_edges_range_t, tchecker::zg::initial_value_t,
                            tchecker::zg::outgoing_edges_value_t>::sst_t;  
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

  sst_t next(tchecker::zg::const_state_sptr_t const & s, tchecker::zg::const_transition_sptr_t const & t){
    tchecker::zg::state_sptr_t nexts = this->_state_allocator.construct();
    tchecker::zg::transition_sptr_t nextt = this->_transition_allocator.construct();
    
    // tchecker::make_range<tchecker::syncprod::outgoing_edges_iterator_t, tchecker::end_iterator_t>();
    //tchecker::zg::outgoing_edges_value_t edges = tchecker::make_range<tchecker::syncprod::edges_iterator_t>(t->vedge().begin(), t->vedge().end());
    //outgoing_edges_range_t edges_range = tchecker::make_range<tchecker::syncprod::outgoing_edges_iterator_t, tchecker::end_iterator_t>(t->vedge().begin(), t->vedge().end());

    // tchecker::zg::outgoing_edges_value_t edges = tchecker::make_range<tchecker::syncprod::edges_iterator_t>(t->vedge().begin(), t->vedge().end());
    // tchecker::state_status_t status = tchecker::zg::next(*this->_system, *nexts, *nextt, *this->_semantics, *this->_extrapolation, edges);
    // return std::make_tuple(status, nexts, nextt);
  }
};

tchecker::tck_tar::zg_t * factory(std::shared_ptr<tchecker::ta::system_t const> const & system,
                                          enum tchecker::zg::semantics_type_t semantics_type,
                                          enum tchecker::zg::extrapolation_type_t extrapolation_type, std::size_t block_size)
{
  std::unique_ptr<tchecker::zg::extrapolation_t> extrapolation{
      tchecker::zg::extrapolation_factory(extrapolation_type, *system)};
  if (extrapolation.get() == nullptr)
    return nullptr;
  std::unique_ptr<tchecker::zg::semantics_t> semantics{tchecker::zg::semantics_factory(semantics_type)};
  return new tchecker::tck_tar::zg_t(system, std::move(semantics), std::move(extrapolation), block_size);
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
  bool first = true;
  for (auto && [key, value] : attr){
    if (!first){
      os << ", ";
    } else {
      first = false;
    }
    os << key << ": \"" << value << "\"";
  }
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
  bool first = true;
  for (auto && [key, value] : attr){
    if (!first){
      os << ", ";
    } else {
      first = false;
    }
    os << key << ": \"" << value << "\" ";
  }
}

// std::shared_ptr<tchecker::tck_tar::graph_t>
// randomized_simulation(tchecker::parsing::system_declaration_t const & sysdecl, std::size_t nsteps)
// {
//   std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{sysdecl}};
//   std::shared_ptr<tchecker::tck_tar::zg_t> zg{
//       tchecker::tck_tar::factory(system, tchecker::zg::STANDARD_SEMANTICS, tchecker::zg::NO_EXTRAPOLATION, 100)};
//   std::shared_ptr<tchecker::tck_tar::graph_t> g{new tchecker::tck_tar::graph_t{zg, 100}};
//   std::vector<tchecker::tck_tar::zg_t::sst_t> v;
//   auto const & z = *zg;

//   srand(time(NULL));

//   zg->initial(v);
//   std::size_t k = tchecker::tck_tar::randomized_select(v);
//   if (k == 1000)
//     return g;
//   tchecker::tck_tar::graph_t::node_sptr_t previous_node = g->add_node(zg->state(v[k]));
//   display(std::cout, z, previous_node->state_ptr());
//   std::cout << "\n\n";
//   v.clear();

//   for (std::size_t i = 0; i < nsteps; ++i) {
//     zg->next(previous_node->state_ptr(), v);

//     std::size_t k = tchecker::tck_tar::randomized_select(v);
//     if (k == 1000)
//       break;
//     tchecker::tck_tar::graph_t::node_sptr_t node = g->add_node(zg->state(v[k]));

//     tchecker::zg::const_transition_sptr_t trans{zg->transition(v[k])};
//     display(std::cout, z, trans);
//     std::cout << "\n";
//     display(std::cout, z, node->state_ptr());
//     if (node->state().zone().is_empty()) {
//       std::cout << "\nempty";
//     }
//     std::cout << "\n\n";

//     g->add_edge(previous_node, node, tchecker::graph::reachability::edge_type_t::EDGE_PARENT, zg->transition(v[k]));
//     v.clear();

//     previous_node = node;
//   }

//   return g;
// }



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

bool operator<(const tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> l1, const tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t>  l2){
  return *l1->ptr() < *l2->ptr(); 
}
inline bool operator<(const std::reference_wrapper<const tchecker::vloc_t> l1, const std::reference_wrapper<const tchecker::vloc_t>  l2){
  return l1.get() < l2.get();
}

void compute_interpolant_automaton(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl, std::string trace,
                                   std::string alphabet,
                                   std::ostream & os)
{
  // sysdecl->insert_process_declaration(nullptr);
  // tchecker::tck_tar::randomized_simulation(*sysdecl, 10);
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{*sysdecl}};

  // system->add_process("ad");
  std::shared_ptr<tchecker::tck_tar::zg_t> zg{
      tchecker::tck_tar::factory(system, tchecker::zg::STANDARD_SEMANTICS, tchecker::zg::EXTRA_LU_PLUS_LOCAL, 100)};
  // std::shared_ptr<tchecker::tck_tar::graph_t> g{new tchecker::tck_tar::graph_t{zg, 100}};
  std::vector<tchecker::tck_tar::zg_t::sst_t> v;

  std::vector<std::string> vtrace;
  boost::algorithm::split(vtrace, trace, boost::is_any_of(" \t"), boost::token_compress_on);
  std::vector<std::string> valphabet;
  boost::algorithm::split(valphabet, alphabet, boost::is_any_of(" \t"), boost::token_compress_on);
  for (auto t : vtrace) {
    if (std::find(valphabet.begin(), valphabet.end(), t) == valphabet.end()){
      throw std::runtime_error("Following letter of the trace is not in the alphabet: " + t);
    }
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
  // FIXME: If there are no transitions with sigma but there are outside of the alphabet, take these until you reach a unique transition with sigma
  bool fsm_feasible = true; // whether the trace is feasible in the underlying fsm
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
      fsm_feasible = false;
      break;
    }
    // The current prefix is already unfeasible, stop here.
    if (states[states.size()-1]->zone().is_empty()){
      break;
    }
  }

  // Display
  display(std::cout, *zg, states[0]);
  std::cout << "\n";
  for(unsigned i = 0; i < trans.size(); i++){
    display(std::cout, *zg, trans[i]);
    std::cout << "\n";
    display(std::cout, *zg, states[i+1]);
    std::cout << "\n";
  }

  bool reachable = (trans.size() == vtrace.size()) && !states[states.size()-1]->zone().is_empty();
  if (reachable){
    os << "REACHABLE true\n";
    os << "0 [";
    display(os, *zg, states[0]);
    os << "]\n";
    for(unsigned i = 0; i < trans.size(); i++){
      os << i << " -> " << i+1 << " [";
      display(os, *zg, trans[i]);
      os << "]\n" << i+1 << " [";
      display(os, *zg, states[i+1]);
      os << "]\n";
    }
  } else {

    if (fsm_feasible){
      // FIXME case where the first symbol cannot be read
      // e.g. broadcast_2_2_c-*
      os << "REACHABLE false\n";
      os << "Interpolant Automaton:\n";
      os << "nb_states: " << (trans.size() + 1) << std::endl;
      os << "init: 0" << std::endl;
      os << "accepting: " << trans.size() << std::endl;
      for (unsigned i = 0; i < trans.size(); i++){
        os << "(" << i << "," << vtrace[i] << "," << i+1 << ")\n";
      }
      unsigned last = trans.size();
      for (auto sigma : valphabet){
        os << "(" << last << "," << sigma << "," << last << ")\n";
      }
      // Simple version of the interpolant automaton
      // if l_i == l_j && Z_i <= Z_j, then 
      //     if i>0 (l_{i-1}, Z_{i-1}) -- e_{i-1} --> (l_j, Z_j)
      //     else (lj,Z_j) becomes initial
      for (unsigned i = 0; i < trans.size(); i++){
        for (unsigned j = 0; j < trans.size(); j++){
          if (i == j ) continue;
          if (states[i]->vloc() == states[j]->vloc() && states[i]->zone() <= states[j]->zone()){
            if (i == 0){
              os << "init: " << j << std::endl;
            } else {
              os << "(" << i-1 << "," << vtrace[i-1] << "," << j << ")\n";            
            }
          }
        }
      }
    } else { // !fsm_feasible
      // In this case, the unfeasible trace is trans + vtrace[trans.size()]
      // That is, there exists no edge labeled by vtrace[trans.size()] from states[states.size()-1].
      os << "REACHABLE false\n";
      os << "Interpolant Automaton:\n";
      os << "nb_states: " << (trans.size() + 2) << std::endl;
      os << "init: 0" << std::endl;
      os << "accepting: " << trans.size()+1 << std::endl;
      for (unsigned i = 0; i < trans.size(); i++){
        os << "(" << i << "," << vtrace[i] << "," << i+1 << ")\n";
      }
      // We add a state last=trans.size()+1, and the transition trans.size() --vtrace[trans.size()]--> last
      unsigned last = trans.size()+1;
      std::string additional_sigma = vtrace[trans.size()];
      os << "(" << last-1 << "," << additional_sigma << "," << last << ")" << std::endl;
      for (auto sigma : valphabet){
        os << "(" << last << "," << sigma << "," << last << ")\n";
      }
      // Consider pair i,j < last.
      for (unsigned i = 0; i < last; i++){
        for (unsigned j = 0; j < last; j++){
          if (states[i]->vloc() == states[j]->vloc()){
            if (i == 0){
              os << "init: " << j << std::endl;
            } else {
              os << "(" << i-1 << "," << vtrace[i-1] << "," << j << ")\n";
            }
          }
        }
      }
      // Consider additional edges to go directly to last via additional_sigma
      for (unsigned i = 0; i < last-1; i++){
        if (states[i]->vloc() == states[states.size()-1]->vloc()){
          os << "(" << i << "," << additional_sigma << "," << last << ")\n";            
        }
      }
    }
  }
  // More complex version of the interpolant automaton
  // For all edges e, and all i,j such that l_i = src(e), l_j = tgt(e),
  // if Post_e(Z_i) <= Z_j, then add i --e--> j
  // This will include the straight-line automaton + the simple version above

  /*
  std::map<std::reference_wrapper<const tchecker::vloc_t>, std::shared_ptr<std::vector<int> > > vloc_to_index;
  // std::map<tchecker::shared_vloc_t const, std::shared_ptr<std::vector<int> > > vloc_to_index;
  for(unsigned int i = 0; i < states.size(); i++){
    //auto const & vloc = states[i]->vloc();
    auto vloc_ref = std::reference_wrapper<const tchecker::vloc_t>(states[i]->vloc());
    if (vloc_to_index.count(vloc_ref) == 0){
      vloc_to_index[vloc_ref] = std::make_shared<std::vector<int> >();
    }
    vloc_to_index[vloc_ref]->push_back(i);
  }
  // For each edge, and i,j such that states[i].vloc == src(e), and states[j].vloc == tgt(e)
  // try to add edge from i to j
  for(unsigned e = 0; e < trans.size(); e++){
    // Edge e is from states[e]->vloc() to states[e+1]->vloc()
    auto const & src_vloc = states[e]->vloc();
    auto const & tgt_vloc = states[e+1]->vloc();
    for(auto i : *vloc_to_index[src_vloc]){
      auto sst = zg->next(states[i], trans[i]);
      if (zg->status(sst) != tchecker::STATE_OK) 
        continue;
      auto const & nextz = zg->state(sst)->zone();
      for (auto j : *vloc_to_index[tgt_vloc]){
        if (nextz <= states[j]->zone()){
          std::cout << i << " -> " << j << "\n";
        }
      }
    }
  }
  */  
}

} // namespace tck_tar
} // namespace tchecker
#endif