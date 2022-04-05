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

#include "tchecker/parsing/parsing.hh"
#include "tchecker/syncprod/system.hh"
#include "tchecker/system/output.hh"
#include "tchecker/system/system.hh"
#include "tchecker/ta/system.hh"
#include "tchecker/utils/log.hh"
#include "tchecker/parsing/declaration.hh"

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
 \class smv_converter_t
 \brief Visitor for declarations
 */
class smv_converter_t : public tchecker::parsing::declaration_visitor_t {
public:
  /*!
   \brief Constructor
   */
  smv_converter_t(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl) : _sysdecl(sysdecl){}

  /*!
   \brief Copy constructor
   */
  smv_converter_t(smv_converter_t const &) = default;

  /*!
   \brief Move constructor
   */
  smv_converter_t(smv_converter_t &&) = default;

  /*!
   \brief Destructor
   */
  virtual ~smv_converter_t() = default;

  /*!
   \brief Assignment operator
   */
  smv_converter_t & operator=(smv_converter_t const &) = default;

  /*!
   \brief Move assignment operator
   */
  smv_converter_t & operator=(smv_converter_t &&) = default;

  /*!
   \brief Visitors
   */
  void visit(tchecker::parsing::system_declaration_t const & d) {}
  void visit(tchecker::parsing::clock_declaration_t const & d) {}
  void visit(tchecker::parsing::int_declaration_t const & d) {}
  void visit(tchecker::parsing::process_declaration_t const & d) {}
  void visit(tchecker::parsing::event_declaration_t const & d) {}
  void visit(tchecker::parsing::location_declaration_t const & d) {}
  void visit(tchecker::parsing::edge_declaration_t const & d) {}
  void visit(tchecker::parsing::sync_declaration_t const & d) {}

  private:
  std::shared_ptr<tchecker::parsing::system_declaration_t> _sysdecl;
};


void output_smv(std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl, std::ostream & os){
  smv_converter_t converter(sysdecl);
  sysdecl->visit(converter);
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
