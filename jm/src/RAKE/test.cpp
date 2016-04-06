#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>

#include <fstream>

using namespace std;

int main()
{
  string str;
  getline (cin,str);
  string command ;
  command = "python rake.py \"" + str + "\"";
  system(command.c_str());

  return 0;
}
