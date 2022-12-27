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
#include "complement.hh"

/*!
 \file tck-convert.cc
 \brief Syntax checking and translation of systems
 */

static struct option long_options[] = {{"smv", no_argument, 0, 's'},
                                       {"complement", no_argument, 0, 'c'},
                                       {"complete", no_argument, 0, 'm'},
                                       {"output", required_argument, 0, 'o'},
                                       {"help", no_argument, 0, 'h'},
                                       {0, 0, 0, 0}};

static char * const options = (char *)"cso:h";

void usage(char * progname)
{
  std::cerr << "Usage: " << progname << " [options] [file]" << std::endl;
  std::cerr << "   -s          convert to smv" << std::endl;
  std::cerr << "   -c          complement given deterministic TA" << std::endl;
  std::cerr << "   -m          render given TA complete by adding a rejecting location with label _reject_" << std::endl;
  std::cerr << "   -o          output file" << std::endl;
  std::cerr << "reads from standard input if file is not provided" << std::endl;
}

static bool convert_smv = false;
static bool complement = false;
static bool complete = false;
static bool help = false;
static std::string output_file;
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
    case 'c':
      complement = true;
      break;
    case 'm':
      complete = true;
      break;
    case 'o':
      output_file = optarg;
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
    if (output_file != "") {
      std::ofstream ofs{output_file};
      if (convert_smv){
        tchecker::tck_convert::output_smv(sysdecl, ofs);
      } else if (complement){
        tchecker::tck_convert::complement(sysdecl, ofs);
      } else if (complete) {
        tchecker::tck_convert::complete(sysdecl, ofs);
      }
    } else {
      if (convert_smv){
        tchecker::tck_convert::output_smv(sysdecl, std::cout);
      } else if (complement){
        tchecker::tck_convert::complement(sysdecl, std::cout);
      } else if (complete){
        tchecker::tck_convert::complete(sysdecl, std::cout);
      }
    }
  }
  catch (std::exception & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
