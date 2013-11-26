/*graph.c*/
#include <stdio.h>
#include <stdlib.h>
#include "graph.h" 

/* Exit function to handle fatal errors*/
void C_err_exit(char* msg)
{
    printf("[Fatal Error]: %s \nExiting...\n", msg);
    exit(1);
}

/* Function to create an adjacency list node*/
adjlist_node_p createNode(int v)
{
    adjlist_node_p newNode = (adjlist_node_p)malloc(sizeof(adjlist_node_t));
    if(!newNode)
        C_err_exit("Unable to allocate memory for new node");
 
    newNode->vertex = v;
    newNode->next = NULL;
 
    return newNode;
}
 
/* Function to create a graph with n vertices; Creates both directed and undirected graphs*/
graph_p C_createGraph(int n, graph_type_e type)
{
    int i;
    graph_p graph = (graph_p)malloc(sizeof(graph_t));
    if(!graph)
        C_err_exit("Unable to allocate memory for graph");
    graph->num_vertices = n;
    graph->type = type;
 
    /* Create an array of adjacency lists*/
    graph->adjListArr = (adjlist_p)malloc(n * sizeof(adjlist_t));
    if(!graph->adjListArr)
        C_err_exit("Unable to allocate memory for adjacency list array");
 
    for(i = 0; i < n; i++)
    {
        graph->adjListArr[i].head = NULL;
    }
 
    return graph;
}
 
/*Destroys the graph*/
void C_destroyGraph(graph_p graph)
{
    if(graph)
    {
        if(graph->adjListArr)
        {
            int v;
            /*Free up the nodes*/
            for (v = 0; v < graph->num_vertices; v++)
            {
                adjlist_node_p adjListPtr = graph->adjListArr[v].head;
                while (adjListPtr)
                {
                    adjlist_node_p tmp = adjListPtr;
                    adjListPtr = adjListPtr->next;
                    free(tmp);
                }
            }
            /*Free the adjacency list array*/
            free(graph->adjListArr);
        }
        /*Free the graph*/
        free(graph);
    }
}
 
/* Adds an edge to a graph*/
void C_addEdge(graph_t *graph, int src, int dest)
{
    /* Add an edge from src to dst in the adjacency list*/
    adjlist_node_p newNode = createNode(dest);
    newNode->next = graph->adjListArr[src].head;
    graph->adjListArr[src].head = newNode;
    graph->adjListArr[src].num_members++;
 
    if(graph->type == UNDIRECTED)
    {
        /* Add an edge from dest to src also*/
        newNode = createNode(src);
        newNode->next = graph->adjListArr[dest].head;
        graph->adjListArr[dest].head = newNode;
        graph->adjListArr[dest].num_members++;
    }
}
 
/* Function to print the adjacency list of graph*/
void C_displayGraph(graph_p graph)
{
    int i;
    for (i = 0; i < graph->num_vertices; i++)
    {
        adjlist_node_p adjListPtr = graph->adjListArr[i].head;
        printf("\n%d: ", i);
        while (adjListPtr)
        {
            printf("%d->", adjListPtr->vertex);
            adjListPtr = adjListPtr->next;
        }
        printf("NULL\n");
    }
}

