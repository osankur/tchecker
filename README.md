# TChecker

[![License](https://img.shields.io/badge/license-MIT-informational.svg)](https://github.com/ticktac-project/tchecker/blob/master/LICENSE)


## Description

This is TChecker, an *open-source* model-checker for timed systems. TChecker is
written in C++17 and provides a library of classes to describe timed systems,
to compute the corresponding timed automata, to compute its semantics, as well
as symbolic representations and algorithms to check requirements over timed
systems. It also comes with tools to run these algorithms from the command
line.

TChecker originates and is still lead by academic research on the verification
of timed systems. It is intended to serve as a plateform to experiment new data
structures and algorithms for the verification and the synthesis of timed
systems. The goal of the project is to implement state-of-the-art algorithms as
well as benchmarks to evaluate these algorithms, and to share then with the
community.

This fork contains some simple additional features and is used in the following paper:

  Ocan Sankur. Timed Automata Verification and Synthesis via Finite Automata Learning. TACAS 2023.

Please refer to the [main repository for TChecker](github.com/ticktac-project/tchecker).

## Installation

Please, refer to [Installation of TChecker](https://github.com/ticktac-project/tchecker/wiki/Installation-of-TChecker) or to file INSTALL.md in the repository.

## Usage

Please, refer to [Using TChecker](https://github.com/ticktac-project/tchecker/wiki/Using-TChecker) or to file doc/usage.md in the repository.

## Credits

The authors of the project are credited in the file AUTHORS in the repository.

## License

TChecker is published under the MIT license reproduced below (and in the file
LICENSE in the repository).

MIT License

Copyright (c) 2019 Bordeaux INP, CNRS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
