#!/bin/bash

cpplint --filter="-build/include_subdir,-build/c++11,-readability/todo,-whitespace/line_length,-runtime/references,-runtime/int" --recursive ./src ./include/ueds-connector ./examples