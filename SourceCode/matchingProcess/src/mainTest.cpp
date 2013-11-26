
#include <iostream>
#include <string>
#include "MyGraph.hpp"

using namespace std;

int main () {

	MyGraph<string> g (6);

	g.addEdge(1,4);
	g.addEdge(0,4);
	g.addEdge(0,1);
	g.addEdge(2,4);
	g.addEdge(1,3);
	g.addEdge(5,0);
	g.setVertexData(1,string("hola"));
	g.setVertexData(4,string("adeu"));
	g.setVertexData(0,string("que tal"));
	g.setVertexData(5,string("hola"));
	g.printGraph();

	cout << endl << "Pattern to look for" << endl;
	MyGraph<string> patern(4);
	patern.addEdge(0,1);
	patern.addEdge(1,2);
	patern.addEdge(3,2);
	patern.setVertexData(0, string("hola"));
	patern.setVertexData(1, string("adeu"));
	patern.setVertexData(2, string("que tal"));
	patern.setVertexData(3, string("hola"));
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
