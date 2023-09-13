#include <algorithm.h>

void Dijkstra(SurfaceMesh* mesh,
              VertexPositionGeometry* geometry,
              size_t s_p,
              size_t e_p,
              std::vector<Vector3>& points,
              std::vector<std::array<size_t, 2>>& edges)
{
    std::priority_queue<node> que;

    std::vector<double> distance(mesh->nVertices(), std::numeric_limits<double>::max());
    std::vector<size_t> v_p(mesh->nVertices(), -1);
    std::vector<size_t> isVisited(mesh->nVertices(), 0);
    v_p[s_p] = s_p;
    distance[s_p] = 0;
    que.push(node(s_p, 0));

    while (!que.empty())
    {
        node temp = que.top();
        que.pop();
        if (isVisited[temp.id])
            continue;
        if (temp.id == e_p)
            break;
        isVisited[temp.id] = 1;
        Vertex v1 = mesh->vertex(temp.id);
        for (auto v2 : v1.adjacentVertices())
        {
            Edge edge = mesh->connectingEdge(v1, v2);
            if (distance[v1.getIndex()] + geometry->edgeLength(edge) < distance[v2.getIndex()])
            {
                distance[v2.getIndex()] = distance[v1.getIndex()] + geometry->edgeLength(edge);
                que.push(node(v2.getIndex(), distance[v2.getIndex()]));
                v_p[v2.getIndex()] = temp.id;
            }
        }
    }

    points.push_back(geometry->inputVertexPositions[mesh->vertex(e_p)]);
    do
    {
        e_p = v_p[e_p];
        points.push_back(geometry->inputVertexPositions[mesh->vertex(e_p)]);
    } while (e_p != v_p[e_p]);

    for (size_t i = 0; i < points.size() - 1; i++)
    {
        edges.push_back({i, i + 1});
    }
    // edges.clear();
    // while (e_p != v_p[e_p])
    // {
    //     edges.push_back({e_p, v_p[e_p]});
    //     e_p = v_p[e_p];
    // }
}