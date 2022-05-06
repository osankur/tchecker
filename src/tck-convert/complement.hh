#ifndef TCHECKER_TCK_CONVERT_COMPLEMENT_HH
#define TCHECKER_TCK_CONVERT_COMPLEMENT_HH

#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "tchecker/expression/typed_expression.hh"
#include "tchecker/parsing/declaration.hh"
#include "tchecker/parsing/parsing.hh"
#include "tchecker/statement/typed_statement.hh"
#include "tchecker/syncprod/system.hh"
#include "tchecker/system/output.hh"
#include "tchecker/system/system.hh"
#include "tchecker/ta/system.hh"
#include "tchecker/utils/log.hh"

namespace tchecker {

namespace tck_convert {

/*!
 \class guard_extractor_t_t
 \brief Visitor pattern for typed expressions
 */
class guard_extractor_t : public tchecker::typed_expression_visitor_t {
public:
  /*!
   \brief Constructor
   */
  guard_extractor_t() = default;

  /*!
   \brief Copy constructor
   */
  guard_extractor_t(guard_extractor_t const &) = default;

  /*!
   \brief Destructor
   */
  virtual ~guard_extractor_t() = default;

  /*!
   \brief Assignment operator
   */
  guard_extractor_t & operator=(guard_extractor_t const &) = default;

  /*!
   \brief Move assignment operator
   */
  guard_extractor_t & operator=(guard_extractor_t &&) = default;

  std::pair<std::vector<std::shared_ptr<tchecker::expression_t>>, std::vector<std::shared_ptr<tchecker::expression_t>>>
  extract(tchecker::typed_expression_t const & e)
  {
    discrete_guard.clear();
    clock_guard.clear();
    _has_seen_clocks = false;
    _has_seen_and = false;
    e.visit(*this);
    if (!_has_seen_and) {
      if (_has_seen_clocks) {
        clock_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.clone()));
      }
      else {
        discrete_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.clone()));
      }
    }
    // std::cout << "Discrete guard: ";
    // for (auto exp : discrete_guard) {
    //   std::cout << exp->to_string() << "\t";
    // }
    // std::cout << "\n";
    // std::cout << "Clock guard: ";
    // for (auto exp : clock_guard) {
    //   std::cout << exp->to_string() << "\t";
    // }
    // std::cout << "\n";
    return std::make_pair(discrete_guard, clock_guard);
  }
  /*!
   \brief Visitors
   */
  void visit(tchecker::typed_int_expression_t const & e) {}
  void visit(tchecker::typed_var_expression_t const & e) {}
  void visit(tchecker::typed_bounded_var_expression_t const & e) {}
  void visit(tchecker::typed_array_expression_t const & e) {}
  void visit(tchecker::typed_par_expression_t const & e) { e.expr().visit(*this); }

  void visit(tchecker::typed_binary_expression_t const & e)
  {
    switch (e.binary_operator()) {
    case tchecker::binary_operator_t::EXPR_OP_LAND:
    //   std::cout << "&& Visiting " << e.to_string() << "\n";
      _has_seen_clocks = false;
      _has_seen_and = false;
    //   std::cout << "\tVisiting left: " << e.left_operand().to_string() << "\n";
      e.left_operand().visit(*this);
    //   std::cout << "\tand: " << _has_seen_and << " clock: " << _has_seen_clocks << "\n";
      if (!_has_seen_and) {
        if (_has_seen_clocks) {
          clock_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.left_operand().clone()));
        }
        else {
          discrete_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.left_operand().clone()));
        }
      }
      _has_seen_clocks = false;
      _has_seen_and = false;
      e.right_operand().visit(*this);
    //   std::cout << "\tVisiting right: " << e.right_operand().to_string() << "\n";
    //   std::cout << "\tand: " << _has_seen_and << " clock: " << _has_seen_clocks << "\n";      
      if (!_has_seen_and){
        if (_has_seen_clocks) {
            clock_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.right_operand().clone()));
        }
        else {
            discrete_guard.push_back(std::shared_ptr<tchecker::expression_t>(e.right_operand().clone()));
        }
      }
      _has_seen_and = true;
      break;
    default:
      break;
    }
  }
  void visit(tchecker::typed_unary_expression_t const & e) { e.operand().visit(*this); }
  void visit(tchecker::typed_simple_clkconstr_expression_t const & e) { _has_seen_clocks = true; }
  void visit(tchecker::typed_diagonal_clkconstr_expression_t const & e) { _has_seen_clocks = true; }
  void visit(tchecker::typed_ite_expression_t const & e) { throw std::runtime_error("ITE expressions are not supported"); }

private:
  bool _has_seen_clocks;
  bool _has_seen_and;
  std::vector<std::shared_ptr<tchecker::expression_t>> clock_guard;
  std::vector<std::shared_ptr<tchecker::expression_t>> discrete_guard;
};

class expression_complementer_t : public tchecker::expression_visitor_t{
public:
  /*!
   \brief Constructor
   */
  expression_complementer_t() = default;

  /*!
   \brief Copy constructor
   */
  expression_complementer_t(expression_complementer_t const &) = default;

  /*!
   \brief Move constructor
   */
  expression_complementer_t(expression_complementer_t &&) = default;

  /*!
   \brief Destructor
   */
  virtual ~expression_complementer_t() = default;

  std::vector<std::shared_ptr<tchecker::expression_t>> get_complement(tchecker::expression_t const & e){
      _complement.clear();
      e.visit(*this);
      return _complement;
  }

  /*!
   \brief Visitors
   */
  void visit(tchecker::int_expression_t const & expr) {}
  void visit(tchecker::var_expression_t const & expr) {}
  void visit(tchecker::array_expression_t const & expr) {}
  void visit(tchecker::par_expression_t const & expr) {
      expr.expr().visit(*this);
  }
  void visit(tchecker::unary_expression_t const & expr) {}
  void visit(tchecker::binary_expression_t const & expr) {
      switch(expr.binary_operator()){
      case tchecker::binary_operator_t::EXPR_OP_LAND:
        throw std::runtime_error("&& is not supposed to occur inside a conjunct");
        break;
      case tchecker::binary_operator_t::EXPR_OP_LT:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_GE, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_LE:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_GT, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_EQ:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_LT, expr.left_operand().clone(), expr.right_operand().clone()));
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_GT, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_NEQ:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_EQ, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_GE:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_LT, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_GT:
        _complement.push_back(std::make_shared<tchecker::binary_expression_t>(tchecker::binary_operator_t::EXPR_OP_LE, expr.left_operand().clone(), expr.right_operand().clone()));
        break;
      case tchecker::binary_operator_t::EXPR_OP_MINUS:
        break;
      case tchecker::binary_operator_t::EXPR_OP_PLUS:
        break;
      case tchecker::binary_operator_t::EXPR_OP_TIMES:
        break;
      case tchecker::binary_operator_t::EXPR_OP_DIV:
        break;
      case tchecker::binary_operator_t::EXPR_OP_MOD:
        break;
      }
  }
  void visit(tchecker::ite_expression_t const & expr) {
    throw std::runtime_error("ITE expressions are not supported");      
  }
private:
  std::vector<std::shared_ptr<tchecker::expression_t>> _complement;
};

/*!
 \class system_complementer_t
 \brief Synchronizes a system of timed processes as a system with a unique synchronized process
 */
class system_complementer_t {
public:
  /*!
   \brief Constructor
   \param system : a system of timed processes
   \param separator : separator for names
   \param process_name : name of synchronized process
   */
  system_complementer_t(std::shared_ptr<tchecker::ta::system_t const> const & system)
      : _system(system),  _product("complemented_" + _system->name(), _system->attributes())
  {
    if (system->processes_count() != 1){
        throw std::runtime_error("Can only complement timed automata with a single process");
    }
    _process_name = system->process_name(0);
    integer_variables();
    clock_variables();
    process();
    locations_edges_events();
  }

  /*!
   \brief Accessor
   \return Synchronized product of given system of timed processes
   */
  tchecker::system::system_t complement() const { return _product; }

private:

  /*!
   \brief Add bounded integer variables to synchronized product
   */
  void integer_variables()
  {
    for (tchecker::intvar_id_t id = 0; id < _system->intvars_count(tchecker::VK_DECLARED); ++id) {
      tchecker::intvar_info_t const & info = _system->integer_variables().info(id);
      _product.add_intvar(_system->intvar_name(id), info.size(), info.min(), info.max(), info.initial_value(),
                          _system->intvar_attributes(id));
    }
  }

  /*!
   \brief Add clock variables to synchronized product
   */
  void clock_variables()
  {
    for (tchecker::clock_id_t id = 0; id < _system->clocks_count(tchecker::VK_DECLARED); ++id) {
      tchecker::clock_info_t const & info = _system->clock_variables().info(id);
      _product.add_clock(_system->clock_name(id), info.size(), _system->clock_attributes(id));
    }
  }

  /*!
   \brief Add process to synchronized product
   */
  void process() { _product.add_process(_process_name); }

  /*!
   \brief Add locations, edges and events to synchronized product
   */
  void locations_edges_events()
  {
    // TODO Check whether we are deterministic
    guard_extractor_t gextractor;
    expression_complementer_t complementer;
    tchecker::process_id_t pid = _product.process_id(_process_name);
    // Add all locations by removing their invariants
    for (auto loc : _system->locations()){
        tchecker::system::attributes_t attr;
        for (auto a : loc->attributes().attributes()){
            if ( a.first != "invariant"){
                attr.add_attribute(a.first, a.second);
            } 
        }
        _product.add_location(pid, loc->name(), attr);
    }

    tchecker::system::attributes_t attr;
    attr.add_attribute("labels","_complement_accept");
    _product.add_location(pid, std::string("_complement_accept"), attr);
    tchecker::loc_id_t accept_loc_id = _product.location(pid, "_complement_accept")->id();

    for (auto edge : _system->edges()){
        auto new_src_id = _product.location(pid, _system->location(edge->src())->name())->id();
        auto new_tgt_id = _product.location(pid, _system->location(edge->tgt())->name())->id();
        auto event_name = _system->event_name(edge->event_id());
        if (!_product.is_event(event_name))
          _product.add_event(event_name);
        tchecker::event_id_t new_event_id = _product.event_id(event_name);
        tchecker::system::attributes_t edge_attr;
        auto invar_str = _system->invariant(edge->src()).to_string();
        // Add existing edge by reinforcing the guard with the invariant
        for (auto a : edge->attributes().attributes()){
            if (a.first == "do"){
                edge_attr.add_attribute(a.first, a.second);
            } else if (a.first == "provided"){
                edge_attr.add_attribute(a.first, a.second + " && " + invar_str);
            }
        }
        _product.add_edge(pid, new_src_id, new_tgt_id, new_event_id, edge_attr);

        // Add edges towards the accepting location with the complement of the guard & invariant.
        auto && [disc_guard, clock_guard] = gextractor.extract(_system->guard(edge->id()));
        auto && [disc_invar, clock_invar] = gextractor.extract(_system->invariant(edge->src()));
        std::vector<std::shared_ptr<tchecker::expression_t>> all_elementary_guards;
        for( auto g : disc_guard){
            all_elementary_guards.push_back(g);
        }
        for( auto g : clock_guard){
            all_elementary_guards.push_back(g);
        }
        for( auto g : disc_invar){
            all_elementary_guards.push_back(g);
        }
        for( auto g : clock_invar){
            all_elementary_guards.push_back(g);
        }
        for (auto eg : all_elementary_guards){
            auto comp_eg = complementer.get_complement(*eg);
            for (auto ceg : comp_eg){
                tchecker::system::attributes_t attr;
                attr.add_attribute("provided", ceg->to_string());
                _product.add_edge(pid, new_src_id, accept_loc_id, new_event_id, attr);
            }
        }
    }
    // Add edges with missing events to the accepting location
    tchecker::system::attributes_t empty_attr;
    for (auto loc : _product.locations()){
        for( tchecker::event_id_t eid = 0; eid < _product.events_count(); eid++){
            if (!_product.outgoing_event(loc->id(), eid)){
                _product.add_edge(pid, loc->id(), accept_loc_id, eid, empty_attr);
            }
        }
    }

  }

  std::shared_ptr<tchecker::ta::system_t const> _system; /*!< System of timed processes */
  std::string _process_name;                                   /*!< Name of synchronized process */
  tchecker::system::system_t _product;                         /*!< Synchronized product of _system */
};


/**
 * @brief Computes the complement of the given single-process deterministic timed automaton.
 * 
 * @param sysdecl input timed automaton
 * @param os 
 * @pre sysdecl is deterministic and contains a single process
 * @post the complement TA was output to os
 */
void complement(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl, std::ostream & os)
{
  auto system = std::make_shared<tchecker::ta::system_t>(*sysdecl);
  guard_extractor_t gextractor;
  expression_complementer_t excomplementer;

  system_complementer_t scomplementer(system);
  auto csys = scomplementer.complement();
  tchecker::system::output_tck(os, csys);
}

} // namespace tck_convert
} // namespace tchecker

#endif