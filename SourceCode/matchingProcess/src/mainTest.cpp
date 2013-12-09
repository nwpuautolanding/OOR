
#include <iostream>
#include <string>
#include "MyGraph.hpp"

using namespace std;

int main () {

	MyGraph<string> g (5);

    //g.addEdge(0,2);
	g.addEdge(2,1);
    g.addEdge(3,2);
    g.addEdge(2,4);
////g.addEdge(1,3);
////g.addEdge(5,0);
	g.setVertexData(0,string("v"));
	g.setVertexData(1,string("v"));
	g.setVertexData(2,string("h"));
	g.setVertexData(3,string("v"));
	g.setVertexData(4,string("v"));
	g.printGraph();

	cout << endl << "Pattern to look for" << endl;
	MyGraph<string> patern(4);
	patern.addEdge(0,1);
	patern.addEdge(2,1);
	patern.addEdge(3,1);
	patern.setVertexData(0, string("v"));
	patern.setVertexData(1, string("h"));
	patern.setVertexData(2, string("v"));
	patern.setVertexData(3, string("v"));
	patern.printGraph();
	
	vector<int> resultIdxList;
	bool result;
	result = g.containsSubgraph(patern, resultIdxList);

	if (result) {
		cout << "Trobat" << endl;
	} else {
		cout << "No trobat" << endl;
	}

	for (vector<int>::const_iterator i = resultIdxList.begin(); i < resultIdxList.end(); ++i) {
		cout << *i << ", " << endl;
	}
}
