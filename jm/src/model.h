#ifndef MODEL_H   
#define MODEL_H
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

void train(vector<int> rgb, vector<string> tokens);

void test(vector<int> rgb, vector<string> tokens);

void learnModel();

#endif
