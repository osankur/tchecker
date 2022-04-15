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
#include "smv.hh"
#include "interpolant.hh"

/*!
 \file tck-convert.cc
 \brief Syntax checking and translation of systems
 */

static struct option long_options[] = {{"smv", no_argument, 0, 's'},
                                       {"interpolate", required_argument, 0, 'i'},
                                       {"help", no_argument, 0, 'h'},
                                       {0, 0, 0, 0}};

static char * const options = (char *)"si:h";
std::string trace = "";
void usage(char * progname)
{
  std::cerr << "Usage: " << progname << " [options] [file]" << std::endl;
  std::cerr << "   -s          convert to smv" << std::endl;
  std::cerr << "   -i trace     compute interpolants for given space-separated trace" << std::endl;
  std::cerr << "reads from standard input if file is not provided" << std::endl;
}

static bool convert_smv = false;
static bool interpolate = false;
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
    case 'i':
      if (strcmp(optarg, "") == 0)
        throw std::invalid_argument("Invalid empty trace");
      interpolate = true;
      trace = optarg;
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
    if (convert_smv){
      tchecker::tck_convert::output_smv(sysdecl, std::cout);
    } else if (interpolate){
      tchecker::tck_interpolate::compute_interpolant_automaton(sysdecl, trace, std::cout);
    }
  }
  catch (std::exception & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
