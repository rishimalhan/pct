#include<vector>
#include<string>
using std::vector;
using std::string;
void print_vec_of_vec(std::vector<std::vector<double>> in_vec);
vector< vector<double> > read_csv(string filename);
void write_csv(string filename, vector< vector<double> > values);
void write_csv_int_values(string filename, vector<vector<int> > values);
vector< vector<int> > read_csv_int_values(string filename);
void write_string(string filename, string text);
std::string read_string(string filename) ;