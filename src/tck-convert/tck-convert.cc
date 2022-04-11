/*
 * This file is a part of the TChecker project.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include <cstring>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <sstream>


#include "tchecker/parsing/parsing.hh"
#include "tchecker/syncprod/system.hh"
#include "tchecker/system/output.hh"
#include "tchecker/system/system.hh"
#include "tchecker/ta/system.hh"
#include "tchecker/utils/log.hh"
#include "tchecker/parsing/declaration.hh"
#include "tchecker/expression/typed_expression.hh"
#include "tchecker/statement/typed_statement.hh"
/*!
 \file tck-syntax.cc
 \brief Syntax checking and translation of systems
 */

static struct option long_options[] = {{"smv", no_argument, 0, 's'},
                                       {"help", no_argument, 0, 'h'},
                                       {0, 0, 0, 0}};

static char * const options = (char *)"sh";

void usage(char * progname)
{
  std::cerr << "Usage: " << progname << " [options] [file]" << std::endl;
  std::cerr << "   -s          convert to smv" << std::endl;
  std::cerr << "reads from standard input if file is not provided" << std::endl;
}

static bool convert_smv = false;
static bool help = false;

int parse_command_line(int argc, char * argv[])
{
  while (true) {
    int c = getopt_long(argc, argv, options, long_options, nullptr);

    if (c == -1)
      break;

    if (c == ':')
      throw std::runtime_error("Missing option parameter");
    else if (c == '?')
      throw std::runtime_error("Unknown command-line option");

    switch (c) {
    case 's':
      convert_smv = true;
      break;
    case 'h':
      help = true;
      break;
    default:
      throw std::runtime_error("I should never be executed");
      break;
    }
  }

  return optind;
}

/*!
 \brief Load system from a file
 \param filename : file name
 \return The system declaration loaded from filename, nullptr if parsing error occurred
*/
std::shared_ptr<tchecker::parsing::system_declaration_t> load_system(std::string const & filename)
{
  tchecker::parsing::system_declaration_t * sysdecl = nullptr;
  try {
    sysdecl = tchecker::parsing::parse_system_declaration(filename);
  }
  catch (std::exception const & e) {
    std::cerr << tchecker::log_error << " " << e.what() << std::endl;
  }

  if (sysdecl == nullptr)
    tchecker::log_output_count(std::cout);

  return std::shared_ptr<tchecker::parsing::system_declaration_t>(sysdecl);
}


/*!
 \class typed_expression_to_smv_t_t
 \brief Visitor pattern for typed expressions
 */
class typed_expression_to_smv_t : public tchecker::typed_expression_visitor_t {
public:
  /*!
   \brief Constructor
   */
  typed_expression_to_smv_t() = default;

  /*!
   \brief Copy constructor
   */
  typed_expression_to_smv_t(typed_expression_to_smv_t const &) = default;

  /*!
   \brief Destructor
   */
  virtual ~typed_expression_to_smv_t() = default;

  /*!
   \brief Assignment operator
   */
  typed_expression_to_smv_t & operator=(typed_expression_to_smv_t const &) = default;

  /*!
   \brief Move assignment operator
   */
  typed_expression_to_smv_t & operator=(typed_expression_to_smv_t &&) = default;

  std::string to_string(tchecker::typed_expression_t const & e){
    _ss.str("");
    e.visit(*this);
    return _ss.str();
  }
  /*!
   \brief Visitors
   */
  void visit(tchecker::typed_int_expression_t const & e) {
    _ss << e.value();
  }
  void visit(tchecker::typed_var_expression_t const & e) {
    _ss << e.to_string();
  }
  void visit(tchecker::typed_bounded_var_expression_t const & e) {
    _ss << e.to_string();
  }
  void visit(tchecker::typed_array_expression_t const & e) {
    _ss << e.to_string();
  }
  void visit(tchecker::typed_par_expression_t const & e) {
    _ss << "(";
    e.expr().visit(*this);
    _ss << ")";
  }
  void output_unary_operator(tchecker::unary_operator_t op){
    switch(op){
      case tchecker::unary_operator_t::EXPR_OP_LNOT:
      _ss << "!";
      break;
      case tchecker::unary_operator_t::EXPR_OP_NEG:
      _ss << "!";
      break;
    }
  }

  void output_binary_operator(tchecker::binary_operator_t op){
    switch(op){
      case tchecker::binary_operator_t::EXPR_OP_LAND:
        _ss << " & ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_LT:
        _ss << " < ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_LE:
        _ss << " <= ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_EQ:
        _ss << " = ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_NEQ:
        _ss << " != ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_GE:
        _ss << " >= ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_GT:
        _ss << " > ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_MINUS:
        _ss << " - ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_PLUS:
        _ss << " + ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_TIMES:
        _ss << " * ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_DIV:
        _ss << " / ";
        break;
      case tchecker::binary_operator_t::EXPR_OP_MOD:
        _ss << " mod ";
        break;
    }
  }


  void visit(tchecker::typed_binary_expression_t const & e) {
    // _ss << "(";
    e.left_operand().visit(*this);
    // _ss << ")";
    output_binary_operator(e.binary_operator());
    // _ss << "(";
    e.right_operand().visit(*this);
    // _ss << ")";
  }
  void visit(tchecker::typed_unary_expression_t const & e) {
    output_unary_operator(e.unary_operator());
    e.operand().visit(*this);
  }
  void visit(tchecker::typed_simple_clkconstr_expression_t const & e) {
    _ss << e.to_string();
  }
  void visit(tchecker::typed_diagonal_clkconstr_expression_t const & e) {
    _ss << e.to_string();
  }
  void visit(tchecker::typed_ite_expression_t const & e) {
    throw std::runtime_error("ITE expressions are not supported");
  }
private:
  std::stringstream _ss;
};


/*!
 \class typed_statement_to_smv_t
 \brief Visitor pattern for typed statements
 */
class typed_statement_to_smv_t : public tchecker::typed_statement_visitor_t {
public:
  /*!
   \brief Constructor
   */
  typed_statement_to_smv_t() = default;

  /*!
   \brief Copy constructor
   */
  typed_statement_to_smv_t(typed_statement_to_smv_t const &) = default;

  /*!
   \brief Move constructor
   */
  typed_statement_to_smv_t(typed_statement_to_smv_t &&) = default;

  /*!
   \brief Destructor
   */
  virtual ~typed_statement_to_smv_t() = default;

  /*!
   \brief Assignment operator
   */
  typed_statement_to_smv_t & operator=(typed_statement_to_smv_t const &) = default;

  /*!
   \brief Move assignment operator
   */
  typed_statement_to_smv_t & operator=(typed_statement_to_smv_t &&) = default;

  std::string to_string(tchecker::typed_statement_t const & st, const std::string & lhs){
    _ss.str("");
    this->_lhs = lhs;
    st.visit(*this);
    return _ss.str();
  }
  /*!
   \brief Visitors
   */
  void visit(tchecker::typed_nop_statement_t const & st) {}
  void visit(tchecker::typed_assign_statement_t const & st) {
    if (st.lvalue().to_string() == this->_lhs){
      _ss << st.rvalue().to_string();
    }
  }
  void visit(tchecker::typed_int_to_clock_assign_statement_t const & st) {
    if (st.clock().to_string() == this->_lhs){
      _ss << st.rvalue().to_string();
    }
  }
  void visit(tchecker::typed_clock_to_clock_assign_statement_t const & st){
    throw std::runtime_error("Clock to clock assignment not supported");
  }
  void visit(tchecker::typed_sum_to_clock_assign_statement_t const & st) {
    throw std::runtime_error("No sum to clock assignment allowed");
  }
  void visit(tchecker::typed_sequence_statement_t const & st) {
    st.first().visit(*this);
    st.second().visit(*this);
  }
  void visit(tchecker::typed_if_statement_t const & st) {
    throw std::runtime_error("No ITE statement allowed");
  }
  void visit(tchecker::typed_while_statement_t const & st) {
    throw std::runtime_error("No while statement allowed");
  }
  void visit(tchecker::typed_local_var_statement_t const & st) {
    throw std::runtime_error("No local var statement allowed");
  }
  void visit(tchecker::typed_local_array_statement_t const & st) {
    throw std::runtime_error("No local array statement allowed");
  }
private:
  std::string _lhs;
  std::stringstream _ss;  
};



void output_smv(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl, std::ostream & os){
  std::stringstream _varDecl;
  std::stringstream _initDecl;
  std::stringstream _edgeComments;
  typed_expression_to_smv_t expressionConverter;
  typed_statement_to_smv_t statementConverter;
  try {
    tchecker::ta::system_t system(*sysdecl);    
    // Locations
    _varDecl << "\t_loc_ : {";
    bool first = true;
    for (auto l : system.locations()){
      if (!first){
        _varDecl << ", ";
      }
      _varDecl << l->name();
      first = false;
    }
    _varDecl << "};\n";
    os << "@TIME_DOMAIN continuous\n";
    os << "MODULE main\n";
    os << "IVAR\n";
    os << "_edge_ : 0.." << system.edges_count()-1 << ";\n";

    // Bounded integer variables
    tchecker::integer_variables_t const & integer_variables = system.integer_variables();
    tchecker::intvar_id_t intvars_count = system.intvars_count(tchecker::VK_DECLARED);
    for (tchecker::intvar_id_t id = 0; id < intvars_count; ++id) {
      tchecker::intvar_info_t const & info = integer_variables.info(id);
      if (info.size() == 1){
        _varDecl << "\t" << integer_variables.name(id) << " : " << info.min() << ".." << info.max() << ";\n";
        _initDecl << "\tinit(" << integer_variables.name(id) << ") := " << info.initial_value() << ";\n";
      } else {
        _varDecl << "\t" << integer_variables.name(id) << " : array 0.." << info.size() << " of " << info.min() << ".." << info.max() << ";\n";
        for (unsigned i = 0; i < info.size(); i++){
          _initDecl << "\tinit(" << integer_variables.name(id) << "[" << i << "]) := " << info.initial_value() << ";\n";
        }
      }
    }

    // Clocks
    tchecker::clock_variables_t const & clock_variables = system.clock_variables();
    tchecker::clock_id_t clocks_count = system.clocks_count(tchecker::VK_DECLARED);
    for (tchecker::clock_id_t id = 0; id < clocks_count; ++id) {
      tchecker::clock_info_t const & info = clock_variables.info(id);
      if (info.size() == 1){
        _varDecl << "\t" << clock_variables.name(id) << " : clock;\n";
        _initDecl << "\tinit(" << clock_variables.name(id) << ") := 0;\n";
      } else {
        throw std::runtime_error("Clock arrays not supported");
        // _varDecl << "\t" << clock_variables.name(id) << " : array 0.." << info.size() << " of clock;\n";
        // for (unsigned i = 0; i < info.size(); i++){
        //   _initDecl << "\tinit(" << clock_variables.name(id) << "[" << i << "]) := 0;\n";
        // }
      }
    }

    os << "VAR\n" << _varDecl.str() << "\n";
    os << "INIT\n\t(";
    first = true;
    for(auto l : system.locations()){
      auto flags = l->attributes().values("initial");
      if(flags.begin() != flags.end()){
        if (!first) {
          os << " | ";
        }
        os << "_loc_ = " << l->name() << " ";
        first = false;
      }
    }
    os << ");\n";
    os << "ASSIGN\n" << _initDecl.str() << "\n";

    // Location updates
    os << "\tnext(_loc_) := case\n";
    for (auto e : system.edges()){
      auto &g = system.guard(e->id());
      std::string guardStr = expressionConverter.to_string(g);
      if ( guardStr == "1"){
        guardStr = "TRUE";
      }
      os << "\t\t_loc_ = " << system.location(e->src())->name() << 
        " & _edge_ = " <<  e->id() <<
        " & " << guardStr << " : " << system.location(e->tgt())->name() + ";\n";
      _edgeComments << "-- _edge_ = " << e->id() << ": " << system.event_name(e->event_id()) << " " 
        "_loc_ = " << system.location(e->src())->name() << " & " << guardStr << "\n";
    }
    os  << "\t\tTRUE : _loc_;\n\tesac;\n";

    // Clock updates
    for (tchecker::clock_id_t id = 0; id < clocks_count; ++id) {
      os << "\tnext(" << clock_variables.name(id) << ") := case\n";
      for (auto e : system.edges()){
        auto &st = system.statement(e->id());
        std::string updateStr = statementConverter.to_string(st, clock_variables.name(id));
          if (updateStr.size() > 0){
          auto &g = system.guard(e->id());
          std::string guardStr = expressionConverter.to_string(g);
          if ( guardStr == "1"){
            guardStr = "TRUE";
          }
          os << "\t\t_loc_ = " << system.location(e->src())->name() << 
            " & _edge_ = " <<  e->id() <<
            " & " << guardStr << " : " << updateStr + ";\n";
          }
      }
      os  << "\t\tTRUE : " << clock_variables.name(id) << ";\n\tesac;\n";
    }

    // Int var updates
    for (tchecker::intvar_id_t id = 0; id < intvars_count; ++id) {
      os << "\tnext(" << integer_variables.name(id) << ") := case\n";
      for (auto e : system.edges()){
        auto &st = system.statement(e->id());
        std::string updateStr = statementConverter.to_string(st, integer_variables.name(id));
          if (updateStr.size() > 0){
          auto &g = system.guard(e->id());
          std::string guardStr = expressionConverter.to_string(g);
          if ( guardStr == "1"){
            guardStr = "TRUE";
          }
          os << "\t\t_loc_ = " << system.location(e->src())->name() << 
            " & _edge_ = " <<  e->id() <<
            " & " << guardStr << " : " << updateStr + ";\n";
          }
      }
      os  << "\t\tTRUE : " << integer_variables.name(id) << ";\nesac;\n";
    }
    // Urgent locations
    os << "\nURGENT\n";
    first = true;    
    for (auto l : system.locations()){
      if(system.is_urgent(l->id())){
        if (!first){
          os << " | ";
        }
        first = false;
        os << "\t(_loc_ = " << l->name() << ")\n";
      }
    }

    // Invariants
    os << "\nINVAR\n";
    first = true;
    for (auto l : system.locations()){
      auto &g = system.invariant(l->id());
      std::string guardStr = expressionConverter.to_string(g);
      if ( guardStr != "1"){
        os << "\t";
        if (!first){
          os << " & ";
        }
        os << "\t(_loc_ = " << l->name() << " -> (" << guardStr << "))\n";
        first = false;
      }
    }
    os << _edgeComments.str() << "\n";
  }
  catch (std::exception & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
  }

  if (tchecker::log_error_count() > 0) {
    tchecker::log_output_count(std::cout);
    return;
  }
}

/*!
 \brief Main function
*/
int main(int argc, char * argv[])
{
  try {
    int optindex = parse_command_line(argc, argv);

    if (argc - optindex > 1) {
      std::cerr << "Too many input files" << std::endl;
      usage(argv[0]);
      return EXIT_FAILURE;
    }

    if (help) {
      usage(argv[0]);
      return EXIT_SUCCESS;
    }

    std::string input_file = (optindex == argc ? "" : argv[optindex]);

    std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl{load_system(input_file)};
    if (sysdecl.get() == nullptr)
      return EXIT_FAILURE;
    output_smv(sysdecl, std::cout);
  }
  catch (std::exception & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
