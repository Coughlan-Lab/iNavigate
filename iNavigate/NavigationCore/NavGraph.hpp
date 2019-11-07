//
//  Graph.hpp
//  GraphNav
//
//  Created by Giovanni Fusco on 3/26/19.
//  Copyright © 2019 Giovanni Fusco. All rights reserved.
//

#ifndef Graph_h
#define Graph_h

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "rapidjson/document.h"
//#include "NavigationTracker.hpp"
#include <fstream>
#include <map>

# define INF 0x3f3f3f3f

namespace navgraph{
    
    
    class NavGraph {
        
    public:
        enum NodeType { Control = 0, Destination = 1, Link = 2, Undef = -1 };
        
        struct Edge{
            float length; // length of the edge in u,v coordinates
            float angleDeg; // orientation of the edge from source to destination
        };
        
        struct Node{
            int id;
            NodeType type;
            cv::Point2f positionUV;
            int floor;
            std::string label;
            bool isDoor;
            std::string comments;
            std::map<int, Edge> edges;
            std::vector<int> paths;
            std::vector<float> dist;
        };
        
        // this structure contains information about where the user is on the graph. It maps a peak location in UV coord. to
        // UV location on the graph.
        struct SnappedPosition{
            cv::Point2f uvPos;
            cv::Point2f realuvPos;
            int srcNodeId; // IDs of the nodes of the edge the user has been snapped on
            int destNodeId;
            float dist2Src; // distance to nodes on the graph
            float dist2Dest;
            float distance;
            double deltaYaw;
//            Node closestNode;
            Node srcNode; // the node at the source of the edge the user is on
            Node destNode; // the node at the end of the edge
            std::string instruction;
            float refAngle; // this angle represents the orientation of the edge from srcNode to destNode
            float heading;
            float headingDiff;
        };
        
        NavGraph() { ; }
        NavGraph(std::string jsonFileName, std::shared_ptr<maps::MapManager> mapManager){
            rapidjson::Document document;
            _mapManager = mapManager;
            std::ifstream infile(jsonFileName);
            std::string json((std::istreambuf_iterator<char>(infile)),
                             std::istreambuf_iterator<char>());
            document.Parse(json.c_str());
            _parseGraphMatrices(document);
            _parseNodes(document);
            _computePaths();
            
        }
        
        inline const std::map<int, NavGraph::Node> getNodes() { return _nodes; }
        inline const Node getNode(int nodeId) { return _nodes[nodeId]; }
        
        std::vector<int> getPathFromCurrentLocation(SnappedPosition spos, int dest){
            std::vector<int> p1 = getPath(spos.srcNodeId, dest);
//            std::cout << "\n";
            std::vector<int> p2 = getPath(spos.destNodeId, dest);
//            std::cout << "\n";
            cv::Point2f diff = spos.uvPos - _nodes[spos.srcNodeId].positionUV;
            float d1 = sqrt(diff.x*diff.x + diff.y*diff.y);
            diff = spos.uvPos - _nodes[spos.destNodeId].positionUV;
            float d2 = sqrt(diff.x*diff.x + diff.y*diff.y);
            
            float c1 = _nodes[spos.srcNodeId].dist[dest] + d1;
            float c2 = _nodes[spos.destNodeId].dist[dest] + d2;
            //        float cmid =
            //std::cout << "c1: " << c1 << "\n";
            //std::cout << "c2: " << c2 << "\n";
            return (c1 < c2) ? p1 : p2;
            
        }
        
        
        std::vector<int> getPath(int src, int dest){
            std::vector<int> path;
            _getPath(this->_nodes[src].paths, dest, path);
            return path;
        }
        
        std::vector<int> getPath(Node src, Node dest){
            std::vector<int> path;
            _getPath(src.paths, dest.id, path);
            return path;
        }
        
        
        // note: uvpos.y is the ascissa, .x the ordinate
        SnappedPosition snapUV2Graph(cv::Point2f uvpos, float deltaYaw, int floor, bool checkWalls = false){
            double minDist = INF;
            cv::Point2f minpos;
            SnappedPosition spos;
            
            // if no location matches, the location stays unchanged
            spos.uvPos = uvpos;
            spos.srcNodeId = -1;
            spos.destNodeId = -1;
            spos.realuvPos = uvpos;
            spos.deltaYaw = deltaYaw;
            for (auto& n : _nodes){
                if (n.second.floor == floor){
                    for (auto& e : n.second.edges){
                        cv::Point2f pt = projectPointToGraph(n.second, _nodes[e.first], uvpos);
                        cv::Point2f diff = uvpos - pt;
                        double d = sqrt(diff.x*diff.x + diff.y*diff.y);
                        
                        if (d < minDist){
                            bool hit = _mapManager->isPathCrossingWalls(_mapManager->uv2pixels(uvpos), _mapManager->uv2pixels(pt));
                            if (!hit){
                                minDist = d;
                                spos.uvPos = pt;
                                spos.srcNodeId = n.first;
                                spos.destNodeId = e.first;
                                spos.srcNode = _nodes[spos.srcNodeId];
                                spos.destNode = _nodes[spos.destNodeId];
                                spos.dist2Src = cv::norm(pt-n.second.positionUV);
                                spos.dist2Dest = cv::norm(pt-_nodes[e.first].positionUV);
                                spos.refAngle = -1e6; //need to compute this based on what is the next node in the path
                                
                                if (spos.dist2Src < spos.dist2Dest)
                                    spos.distance = cv::norm(uvpos-n.second.positionUV);
                                else
                                    spos.distance = cv::norm(uvpos -_nodes[e.first].positionUV);
                            }
                        }
                    }
                }
            }
            return spos;
        }
        
        //adapted from http://www.alecjacobson.com/weblog/?p=1486
        cv::Point2f projectPointToGraph(Node n1, Node n2, cv::Point2f pt){
            cv::Point2f p1 = n1.positionUV;
            cv::Point2f p2 = n2.positionUV;
            // vector from p1 to p2
            cv::Point2f diff = p2 - p1;
            float diff_squared = diff.dot(diff);
            if (diff_squared == 0)
                return p1;
            else{
                //vector from A to p
                cv::Point2f n1toPt = pt - p1;
                //  from http://stackoverflow.com/questions/849211/
                //  Consider the line extending the segment, parameterized as A + t (B - A)
                //  We find projection of point p onto the line.
                //  It falls where t = [(pt-n1) . (n2-n1)] / |n2-n1|^2
                float t = n1toPt.dot(diff)/diff_squared;
                if (t < 0.0)
                    //  "Before" p1 on the line, just return p1
                    return p1;
                else if (t > 1.)
                    // "After" p2 on the line, just return p2
                    return p2;
                else
                    return p1 + t * diff;
            }
        }
        
        
        cv::Mat plotGraph(int floor, bool pause = false){
            // plot graph relative to the specified floor
            cv::Mat map = _mapManager->getWallsImageRGB();
            for (const auto& n : _nodes ){
                if (n.second.floor == floor){
                    cv::Point2i pt = _mapManager->uv2pixels(n.second.positionUV);
                    cv::circle(map, cv::Point2i(pt.y, pt.x), 3, getNodeColor(n.second.type));
                    cv::putText(map, std::to_string(n.first), cv::Point2i(pt.y, pt.x), cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(0,255,0));
                }
            }
            return map;
        }
        
        
        inline cv::Point2f toUVOrigin(cv::Point2f uv){ //sets origin in lower left corner
            return cv::Point2f(uv.x, _mapManager->getMapSizePixels().height - uv.y);
        }
        
        
        cv::Scalar getNodeColor(NodeType type){
            switch (type){
                case NodeType::Control:
                    return cv::Scalar(0,255,0);
                case NodeType::Destination:
                    return cv::Scalar(255,0,0);
                case NodeType::Link:
                    return cv::Scalar(0,0,255);
                default:
                    return cv::Scalar(0,255,0);
            }
        }
        
        
        void showClosestNodeToPoint(int floor, bool checkWalls = false){
            cv::Mat map = plotGraph(floor);
            cv::Point2i pt;
            
            SnappedPosition snap = snapUV2Graph(_mapManager->pixels2uv(pt), floor, false);
            std::vector<int> p = getPathFromCurrentLocation(snap, 8);
            cv::Point2i ptpx = _mapManager->uv2pixels(snap.uvPos);
            cv::circle(map, cv::Point2i(ptpx.y, ptpx.x), 3, cv::Scalar(0,255,255));
        }
        
        // for debugging
        static void onMouse( int event, int x, int y, int, void* ptr)
        {
            if( event != cv::EVENT_LBUTTONDOWN )
                return;
            cv::Point* p = (cv::Point*)ptr;
            p->x = y;
            p->y = x;
//            std::cerr << x << "," << y << "\n";
        }
        
        int findClosestNodeId(cv::Point2f pos, int floor, bool checkWalls = false){
            int id = -1;
            float minDist = 1e6;
            float d;
            for (const auto& n : _nodes){
                if (n.second.floor == floor){
                    cv::Point2f diff = pos - n.second.positionUV;
                    d = (diff.x*diff.x + diff.y*diff.y);
                    if (d < minDist){
                        minDist = d;
                        id = n.first;
                    }
                }
                
            }
            return id;
        }
        
        Node getClosestNode(cv::Point2f pos, int floor, bool checkWalls = false){
            int nodeId = findClosestNodeId(pos, floor, checkWalls);
            return getNode(nodeId);
        }
        
    private:
        cv::Mat _weights;
        cv::Mat _angles;
        std::map<int, NavGraph::Node> _nodes;
        std::shared_ptr<maps::MapManager> _mapManager;
        
        // load weights and angle matrices
        void _parseGraphMatrices(const rapidjson::Document &document){
            const rapidjson::Value& w = document["weights"];
            const rapidjson::Value& a = document["angles"];
            _weights = cv::Mat::zeros(a.Size(), a.Size(), CV_32F);
            _angles = cv::Mat::zeros(a.Size(), a.Size(), CV_32F);
            for (rapidjson::SizeType i = 0; i < a.Size(); i++){
                float* Wi = _weights.ptr<float>(i);
                float* Ai = _angles.ptr<float>(i);
                const rapidjson::Value& ai = a[i];
                const rapidjson::Value& wi = w[i];
                for (rapidjson::SizeType j = 0; j < ai.Size(); j++){
                    Wi[j] = wi[j].GetFloat();
                    Ai[j] = ai[j].GetFloat();
                    if (Ai[j] < 0)
                        Ai[j] += 360;
                }
            }
        }
        
        void _parseNodes(const rapidjson::Document &document){
            const auto& nodes = document["nodes"].GetObject();
            for(rapidjson::Value::ConstMemberIterator it=nodes.MemberBegin(); it != nodes.MemberEnd(); it++) {
                
                Node node;
                int nodeId = it->value["id"].GetInt()-1;
                node.id = nodeId;
                std::string ntype = it->value["type"].GetString();
                if (ntype.compare("control") == 0)
                    node.type = NodeType::Control;
                else if (ntype.compare("destination") == 0)
                    node.type = NodeType::Destination;
                else if (ntype.compare("link") == 0)
                    node.type = NodeType::Link;
                
                //            node.type = it->value["type"].GetString();
                node.floor = std::stoi(it->value["floor"].GetString());
                node.label = it->value["label"].GetString();
                node.isDoor = it->value["isDoor"].GetInt();
                node.comments = it->value["comments"].GetString();
                
                const auto& pos = it->value["position"].GetArray();
                node.positionUV = _mapManager->pixels2uv(cv::Point2i(pos[1].GetFloat(), pos[0].GetFloat()));
                
                float* wi = _weights.ptr<float>(nodeId);
                float* ai = _angles.ptr<float>(nodeId);
                
                for (int j = 0; j < _weights.cols; j++){
                    if (wi[j] > 0){
                        Edge e = { .length = wi[j] * static_cast<float>(_mapManager->getScale(node.floor)), .angleDeg = ai[j]};
                        node.edges.insert({j, e});
                    }
                }
                _nodes.insert({nodeId, node});
                
            }
        }
        
        // Funtion that implements Dijkstra's
        // single source shortest path
        // algorithm for a graph represented
        // using adjacency matrix representation
        std::vector<int> _computePathsDjikstra(int src, std::vector<float>& dist){
            int v = static_cast<int>(_nodes.size());
            // The output array. dist[i]
            // will hold the shortest
            // distance from src to i
            
            std::vector<bool> sptSet (v, false);
            
            // Parent array to store shortest path tree
            std::vector<int> parent(v, -1);
            
            dist[src] = 0;
            
            // Find shortest path for all vertices
            for (int count = 0; count < v - 1; count++){
                int u = _minDistance(dist, sptSet);
                sptSet[u] = true;
                
                float* w_u = _weights.ptr<float>(u);
                for (int v = 0; v < _weights.cols; v++){
                    if (!sptSet[v] && w_u[v] && dist[u] + w_u[v] < dist[v]){
                        parent[v] = u;
                        dist[v] = dist[u] + w_u[v];
                    }
                }
            }
            return parent;
        }
        
        int _minDistance(const std::vector<float>& dist, const std::vector<bool>& sptSet) {
            
            // Initialize min value
            float min = std::numeric_limits<float>::max();
            int min_index = -1;
            for (int v = 0; v < dist.size(); v++)
                if (sptSet[v] == false && dist[v] <= min){
                    min = dist[v];
                    min_index = v;
                }
            
            return min_index;
        }
    
        void _getPath(const std::vector<int>& parent, int j, std::vector<int> &path){
            if (parent[j] == - 1){
                path.push_back(j);
                return;
            }
            _getPath(parent, parent[j], path);
            path.push_back(j);
        }
        
        
        void _computePaths(){
            int v = _weights.cols;
            for (auto &n : _nodes){
                std::vector<float> dist (v, std::numeric_limits<float>::max());
                n.second.paths = _computePathsDjikstra(n.first, dist);
                n.second.dist = dist;
            }
        }
        
    };
    
} // end navgraph namespace

#endif /* Graph_h */
