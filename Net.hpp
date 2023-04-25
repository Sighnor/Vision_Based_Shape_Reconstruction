#include <stdio.h>
#include <fstream>
#include <string.h>
#include <vector>
#include <random>
#include "Math.hpp"

class Net_Node
{
    public:
        int id;
        array1d<int> neighbor_ids;
        float weight;
        vec2<float> data;

        Net_Node(int _id, array1d<int> _neighbor_ids) : id(_id), neighbor_ids(_neighbor_ids), weight(), data() {}
};

class Net
{
public:
    int size;
    std::vector<Net_Node> nodes;

    Net(int _size = 0) : size(_size) {}
    
    inline std::vector<vec2<float>> Node_Data();
    inline void Read(const char* filename);
    inline void Write(const char* filename);
    inline void SOM_Init(int width, int height);
    inline int SOM_Find_Min(vec2<float> &contour);
    inline void SOM_Update(vec2<float> &contour, int id, float alpha, float sigma);
};

inline std::vector<vec2<float>> Net::Node_Data()
{
    std::vector<vec2<float>> res;
    for(auto pos : nodes)
    {
        res.push_back(pos.data);
    }
    return res;
}

inline void Net::Read(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    assert(fp != NULL);

    fread(&size, sizeof(int), 1, fp);

    for(int i = 0;i < size;i++)
    {
        int id;
        int neighbor_size;
        array1d<int> neighbor_ids;

        fread(&id, sizeof(int), 1, fp);
        fread(&neighbor_size, sizeof(int), 1, fp);
        neighbor_ids.resize(neighbor_size);
        fread(neighbor_ids.data, sizeof(int), neighbor_ids.size, fp);
        //std::cout << id << ", " << neighbor_ids.size << ", " << neighbor_ids << std::endl;

        Net_Node node(id, neighbor_ids);
        nodes.push_back(node);
    }

    fclose(fp);
}

inline void Net::Write(const char* filename)
{
    int data[] = {7, 
                  0, 1, 1, 
                  1, 2, 0, 2,
                  2, 2, 1, 3,
                  3, 2, 2, 4,
                  4, 2, 3, 5,
                  5, 2, 4, 6,
                  6, 1, 5};
    FILE* fp = fopen(filename, "wb");
    assert(fp != NULL);
    size_t num = fwrite(data, sizeof(int), 27, fp);
    assert((int)num == 27);
    fclose(fp);
}

inline void Net::SOM_Init(int height, int width)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    for(int i = 0;i < size;i++)
    {
        nodes[i].weight = dist(rng);
        //nodes[i].data = vec2(float(0.5 * width), float(float(i + 1) / (size + 1) * height));
        nodes[i].data = vec2(float(float(i + 1) / (size + 1) * width), float(0.5 * height));
        //std::cout << node.id << ", " << node.neighbor_ids << ", " << node.weight << ", " << node.data << std::endl;
    }
}

inline int Net::SOM_Find_Min(vec2<float> &contour)
{
    int id;
    float min_distance = 10000000000;
    for(int i = 0;i < size;i++)
    {
        float temp_distance = distance(nodes[i].data, contour);
        if(temp_distance < min_distance)
        {
            id = i;
            min_distance = temp_distance;
        }
    }
    return id;
}

inline void Net::SOM_Update(vec2<float> &contour, int id, float alpha, float sigma)
{
    std::vector<int> map(size);

    map[id] = 1;
    nodes[id].data = nodes[id].data + (alpha * (contour - nodes[id].data));
    nodes[id].weight = nodes[id].weight;

    if(sigma >= 1.0)
    {
        for(int i = 0;i < nodes[id].neighbor_ids.size;i++)
        {
            int neighbor_id = nodes[id].neighbor_ids(i);

            map[neighbor_id] = 1;
            nodes[neighbor_id].data = nodes[neighbor_id].data + (alpha * (nodes[id].data - nodes[neighbor_id].data));
            nodes[neighbor_id].weight = nodes[neighbor_id].weight;

            if(sigma >= 1.5)
            {
                for(int j = 0;j < nodes[neighbor_id].neighbor_ids.size;j++)
                {
                    int next_id = nodes[neighbor_id].neighbor_ids(j);

                    if(map[next_id] != 1)
                    {
                        nodes[next_id].data = nodes[next_id].data + (alpha * (nodes[id].data - nodes[next_id].data));
                        nodes[next_id].weight = nodes[next_id].weight;
                    }
                }
            }
        }
    }
}