#include "model.h"
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
using namespace std;
#include <set>

string dataFile = "/home/kamala/catkin_ws/src/jm/src/trainArff.txt";
string arffName = "/home/kamala/catkin_ws/src/jm/src/jmModel.arff";
string modelName = "/home/kamala/catkin_ws/src/jm/src/jm.model";

string classifier = "weka.classifiers.functions.Logistic";
string wekaLoc = "/home/kamala/catkin_ws/src/jm/src/weka.jar";

string results = "/home/kamala/catkin_ws/src/jm/src/testResults.log"; 

string testArffName = "/home/kamala/catkin_ws/src/jm/src/test.arff";

string arffhead(vector<string> classNames) {
   string arf ("@RELATION jmmodel \n\n@attribute 'r' numeric\n@attribute 'g' numeric\n@attribute 'b' numeric\n@attribute 'class' {");
   set<string> clsNames(classNames.begin(), classNames.end());
   ostringstream dt;
   for (set<string>::iterator it = clsNames.begin() ; it != clsNames.end(); ++it) {
      string sth;
      sth = *it;
      if(sth != "")
         dt << *it << ",";
   }
   arf += dt.str();
   arf.erase(arf.end() - 1,arf.end());   
   arf +=  "}\n";

   arf += "\n@data\n";

  return arf;

}

string arffAdd(vector<int> rgb, vector<string> tokens) {
   string data("");
   for (std::vector<string>::iterator it = tokens.begin() ; it != tokens.end(); ++it) {
       ostringstream dt;
       dt << "\n";
       if(*it != "") {
        
   	for (std::vector<int>::iterator itb = rgb.begin() ; itb != rgb.end(); ++itb) {
          dt << *itb;
          dt << ",";
        }
        data += dt.str();
        data.erase(data.end() - 1,data.end());
        data += ",";
        string cls = *it;
        boost::erase_all(cls, ",");
        data += cls;
       }  
    }
    return data;


}

void prepareData(vector<int> rgb, vector<string> tokens) {
    string data = arffAdd(rgb,tokens) ;
    fstream fs;
    fs.open (dataFile,fstream::app);
    fs << data;
    fs.close();
}

vector<string> split_string(const string& str,
                                      const string& delimiter)
{
    vector<string> strings;

    string::size_type pos = 0;
    string::size_type prev = 0;
    while ((pos = str.find(delimiter, prev)) != string::npos)
    {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }

    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));

    return strings;
}

vector<string> getClassNames() {
    fstream fs(dataFile,fstream::in);
    vector<string> classNames;
    string line;
    getline(fs,line,'\0');
    vector <string> cls = split_string(line,"\n");
    for (std::vector<string>::iterator itb = cls.begin() ; itb != cls.end(); ++itb) {
           string temp;
           temp = *itb;

           if(temp != "") {
              vector<string> chs ;
              chs = split_string(temp,",");
              string name = chs.back();
              classNames.push_back(name);
        }
    }
    return classNames;

}

void preparetestArff(vector<int> rgb, vector<string> tokens) {
    prepareData(rgb,tokens);
    ofstream ofs;
    ofs.open (testArffName, ofstream::trunc);

    fstream fs(dataFile,fstream::in);
    vector<string> classNames;
    classNames = getClassNames();
    string arf = arffhead(classNames);
    ofs << arf;
    ofs << "\n";
    ostringstream dt;   
    for (std::vector<int>::iterator itb = rgb.begin() ; itb != rgb.end(); ++itb) {
          dt << *itb;
          dt << ",";
    }
    string data;
    data = dt.str();
    cout << data;
    for (vector<string>::iterator itb = classNames.begin() ; itb != classNames.end(); ++itb) {
	  ostringstream dat;
          dat << *itb;
          string value;
          value = dat.str();
          boost::erase_all(value, ",");
          ofs << data << value << endl;
    }
    ofs.close();
}
void prepareArff(vector<int> rgb, vector<string> tokens) {
    prepareData(rgb,tokens);
    
    ofstream ofs;
    ofs.open (arffName, ofstream::trunc);
    
    vector<string> existingValues;
    vector<string> completeDB;
    vector<string> classNames;
    fstream fs(dataFile,fstream::in);
    string line;
    getline(fs,line,'\0');
    vector <string> cls = split_string(line,"\n");
    for (std::vector<string>::iterator itb = cls.begin() ; itb != cls.end(); ++itb) {
           string temp;
           temp = *itb;

           if(temp != "") {
              completeDB.push_back(temp);

              vector<string> chs ;
              chs = split_string(temp,",");
              string name = chs.back();
              classNames.push_back(name);


              for (std::vector<string>::iterator it = existingValues.begin() ; it != existingValues.end(); ++it) {
                  string ext;
                  ext = *it;
                  ext += name;
                  completeDB.push_back(ext);
	      }

              boost::erase_all(temp,name);
              existingValues.push_back(temp);
           }     
    }
    string arf = arffhead(classNames);
    ofs << arf;
    ofs << "\n";
    for (vector<string>::iterator it = completeDB.begin() ; it != completeDB.end(); ++it) {
       string ext;
       ext = *it;
       ofs << ext;
       ofs << "\n";
    }
    fs.close();

    ofs.close();
    
}

void saveModel () {
  ostringstream sM;
  sM << "java -cp ";
  sM << wekaLoc;
  sM << " ";
  sM << classifier;
  sM << " -x 2 -t ";
  sM << arffName;
  sM << " -d ";
  sM << modelName;
  sM << " > saveResults.log";
  int ret = system(sM.str().c_str());
}

void testModel(string testarfName) {
  ostringstream sM;
  sM << "java -cp ";
  sM << wekaLoc;
  sM << " ";
  sM << classifier;
  sM << " -l ";
  sM << modelName;
  sM << " -T ";
  sM << testarfName;
  sM << " > ";
  sM << results;
  int ret = system(sM.str().c_str());
  fstream fs(results,fstream::in);
  string line;
  getline(fs,line,'\0');
  cout << line;
  fs.close();
  cout << "Results are saved in " << results << " ...." << endl;;
}

void train(vector<int> rgb, vector<string> tokens) {
   prepareArff(rgb,tokens);
 //  saveModel();
}

void test(vector<int> rgb, vector<string> tokens) {
//   prepareArff(rgb,tokens);
 //  saveModel();
   preparetestArff(rgb,tokens);
   testModel(testArffName);
}

void learnModel() {
   saveModel();
}
