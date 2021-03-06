#ifndef MY_GRAPH
#define MY_GRAPH MY_GRAPH

#include <iostream>
#include <vector>

extern "C" {
#include "graph.h"
}

template <typename T>
class MyGraph
{
public:
	MyGraph (int size);
	virtual ~MyGraph ();

	void addEdge (int edge1idx, int edge2idx);
	void printGraph ();

	// Setters
	void setVertexData (int vertexIdx, T data);

	// Getters
	T getVertexData (int vertexIdx) const;
	int getNumVertex() const;
	int getNumEdgesInVertex(int vertexIdx) const;

	bool containsSubgraph (const MyGraph<T> &subgraph, std::vector<int> &resultSubgraphIdx);
	bool containsSubgraph_inmersion (
		const MyGraph<T> &subgraph,
		int graphIdx,
		int subgraphIdx,
		std::vector<bool> graphVisited,
		std::vector<bool> subgraphVisited,
		std::vector<int> &resultSubgraphIdx);

private:
	graph_p graphInstance;

	// Vertex data
	std::vector <T> vertexData;
};


template <typename T>
MyGraph<T>::MyGraph (int size) {

	graphInstance = C_createGraph(size, UNDIRECTED);
	vertexData.resize(size);
}

template <typename T>
MyGraph<T>::~MyGraph() {

	C_destroyGraph(graphInstance);
}

template <typename T>
void MyGraph<T>::addEdge(int edge1idx, int edge2idx) {

	if (edge1idx < graphInstance->num_vertices && edge2idx < graphInstance->num_vertices) {
		C_addEdge (graphInstance, edge1idx, edge2idx);
	} else {
		std::cerr << "Vertex indexes are over the numver of vertices" << std::endl;
	}
}


template <typename T>
void MyGraph<T>::printGraph() {

	C_displayGraph(graphInstance);
}

/*
 * Setters
 */
template <typename T>
void MyGraph<T>::setVertexData(int vertexIdx, T data) {
	
	if (vertexIdx < graphInstance->num_vertices) {
		vertexData[vertexIdx] = data;
	}
}

/*
 * Getters
 */
template <typename T>
T MyGraph<T>::getVertexData (int vertexIdx) const {

	T tmpData;
	if (vertexIdx < graphInstance->num_vertices) {
		tmpData = vertexData[vertexIdx];
	} else {
		std::cerr << "Index out of bounfaries while getVertexData" << std::endl;
	}

	return tmpData;
}

template <typename T>
int MyGraph<T>::getNumVertex() const {
	return graphInstance->num_vertices;
}

template <typename T>
int MyGraph<T>::getNumEdgesInVertex(int vertexIdx) const {
	return graphInstance->adjListArr[vertexIdx].num_members;
}

/*
 * Algorithms
 */
template <typename T>
bool MyGraph<T>::containsSubgraph (const MyGraph<T> &subgraph, std::vector<int> &resultSubgraphIdx) {

	bool result;
	for (size_t i = 0; i < getNumVertex(); ++i) {
		std::vector<bool> graphVisited;
        graphVisited.assign(getNumVertex(), false);
		std::vector<bool> subgraphVisited;
        subgraphVisited.assign(subgraph.getNumVertex(), false);

		std::cout << "restarting search with index: " << i << std::endl;
		result = containsSubgraph_inmersion (subgraph, i, 0,  graphVisited, subgraphVisited, resultSubgraphIdx);

		if (result && resultSubgraphIdx.size() == subgraph.getNumVertex()) {
            std::cout << "trobat!!!" << std::endl;
			break;
		} else {
			std::cout << "Trobat pero no complet with size: " << resultSubgraphIdx.size() << 
            " but expected : " << subgraph.getNumVertex() << std::endl;

			resultSubgraphIdx.clear();
		}
	}
	return result;
}

template <typename T>
bool MyGraph<T>::containsSubgraph_inmersion (
		const MyGraph<T> &subgraph,
		int graphIdx,
		int subgraphIdx,
		std::vector<bool> graphVisited,
		std::vector<bool> subgraphVisited,
		std::vector<int> &resultSubgraphIdx) {

    //bool result = false;
    //std::cout << "Graph visited " << graphVisited << std::endl;
    //std::cout << "Subgraph visietd" << subgraphVisited << std::endl;

        graphVisited[graphIdx] = true;
        subgraphVisited[subgraphIdx] = true;

        std::cout << std::endl << "Graph idx: " << graphIdx << " with type: " << vertexData[graphIdx] << std::endl;
        std::cout << "SubGraph idx: " << subgraphIdx << " with type: " << subgraph.vertexData[subgraphIdx] << std::endl;


        bool result = getVertexData(graphIdx) == subgraph.getVertexData(subgraphIdx);

        if ( result ) {

            adjlist_node_p subgraphNodeTmp = subgraph.graphInstance->adjListArr[subgraphIdx].head;
            for (size_t j = 0; j < subgraph.getNumEdgesInVertex(subgraphIdx); ++j) {
                
                int subgraphIdx_new = subgraphNodeTmp->vertex;
                subgraphNodeTmp = subgraphNodeTmp->next;

                std::cout << "subGraph: " << j << " " << subgraphIdx << "->" << subgraphIdx_new << std::endl;
                if (!subgraphVisited[subgraphIdx_new]) {
                    bool nestResult = false;
                    adjlist_node_p graphNodeTmp = graphInstance->adjListArr[graphIdx].head;
                    for (size_t i = 0; i < getNumEdgesInVertex(graphIdx); ++i) {

                            int graphIdx_new = graphNodeTmp->vertex;
                            graphNodeTmp = graphNodeTmp->next;

                            std::cout << "-----Graph: " << i << " "  << graphIdx << "->" << graphIdx_new << std::endl;
                            if (!graphVisited[graphIdx_new]) {
                                // Make recursive call
                                nestResult =  containsSubgraph_inmersion(subgraph,
                                                graphIdx_new,
                                                subgraphIdx_new,
                                                graphVisited,
                                                subgraphVisited,
                                                resultSubgraphIdx);
                            } else {
                                std::cout << "already visited" << std::endl;
                            }

                            
                            if (nestResult) {
                                graphVisited[graphIdx_new] = true;
                                subgraphVisited[subgraphIdx_new] = true;
                                break;
                            }
                            
                    }
                    result = nestResult && result;
                }

                  //if (result) {
                  //    break;
                  //}
            }
        }
        
        if (result) {
            std::cout << "Adding element: " << graphIdx << std::endl;
            resultSubgraphIdx.push_back(graphIdx);
        }

    std::cout << result << std::endl;
	return result;
}

#endif
