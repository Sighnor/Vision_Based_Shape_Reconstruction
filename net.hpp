#ifndef IMG_NET
#define IMG_NET

#include <stdio.h>
#include <fstream>
#include <string.h>
#include <vector>
#include <random>
#include "array.hpp"
#include "vec.hpp"

class Net_Node
{
    public:
        int id;
        array1d<int> neighbor_ids;
        vec2 weight;

        Net_Node(int _id, array1d<int> _neighbor_ids) : id(_id), neighbor_ids(_neighbor_ids), weight() {}
};

class Net
{
public:
    int size;
    array1d<Net_Node> nodes;

    Net(int _size = 0) : size(_size) {}
    
    inline std::vector<vec2> Nodes_Weights();
    inline void Read(const char* filename);
    inline void Write(const char* filename, int size);
    inline void SOM_Init(int height, int width, std::vector<vec2> &contour_points);
    inline int SOM_Find_Min(vec2 &contour);
    inline void SOM_Update(vec2 &contour, int id, float alpha, float sigma);
};

inline std::vector<vec2> Net::Nodes_Weights()
{
    std::vector<vec2> res;
    for(int i = 0; i < size; i++)
    {
        res.push_back(nodes(i).weight);
    }
    return res;
}

inline void Net::Read(const char* filename)
{
    FILE* fp = fopen(filename, "r");
    assert(fp != NULL);

    fread(&size, sizeof(int), 1, fp);
    nodes.resize(size);

    for(int i = 0; i < size; i++)
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
        nodes(i) = node;
    }

    fclose(fp);
}

inline void Net::Write(const char* filename, int size)
{
    int *data;
    int data_size = 1 + size * 4 - 1 - 1;
    data = (int*)malloc(data_size * sizeof(int));
    
    data[0] = size;

    data[1 + 0] = 0;
    data[1 + 1] = 1;
    data[1 + 2] = 1;

    for(int i = 1; i < size - 1; i++)
    {
        data[1 + i * 4 - 1 + 0] = i;
        data[1 + i * 4 - 1 + 1] = 2;
        data[1 + i * 4 - 1 + 2] = i - 1;
        data[1 + i * 4 - 1 + 3] = i + 1;
    }

    data[1 + (size - 1) * 4 - 1 + 0] = size - 1;
    data[1 + (size - 1) * 4 - 1 + 1] = 1;
    data[1 + (size - 1) * 4 - 1 + 2] = size - 2;

    for(int i = 0; i < data_size; i++)
    {
        // std::cout << data[i] << std::endl;
    }

    FILE* fp = fopen(filename, "wb");
    assert(fp != NULL);
    size_t num = fwrite(data, sizeof(int), data_size, fp);
    assert((int)num == data_size);
    fclose(fp);
}

inline void Net::SOM_Init(int height, int width, std::vector<vec2> &contour_points)
{
    vec2 bottom_point;
    for(int i = 0; i < 20; i++)
    {
        int id = contour_points.size() - 1 - i;
        bottom_point = bottom_point + 0.05f * contour_points[id];
    }

    int begin_id = 0;
    int end_id = 0;
    float min_length = 1000000;
    for(int i = 0; i < 20; i++)
    {
        int id = contour_points.size() - 1 - i;
        float temp_length = length(contour_points[id] - bottom_point);
        if(temp_length < min_length)
        {
            begin_id = id;
            min_length = temp_length;
            nodes(0).weight = contour_points[id];
        }
    }

    array2d<int> map_img_id(height, width);
    array1d<int> sort_id(contour_points.size());
    int curr_id = begin_id;
    vec2 curr_vel = vec2(1, 0);

    map_img_id.set(-1);
    for(int i = 0; i < contour_points.size(); i++)
    {
        map_img_id(contour_points[i].y, contour_points[i].x) = i;
    }
    for(int i = 0; i < contour_points.size(); i++)
    {
        sort_id(i) = curr_id;
        int x = contour_points[curr_id].x;
        int y = contour_points[curr_id].y;
        int next_id;
        vec2 next_vel;
        float best_theta = PI;
        for(int j = -1; j <= 1; j++)
        {
            for(int k = -1; k <= 1; k++)
            {
                int temp_id = map_img_id(y + k, x + j);
                vec2 temp_vel = vec2(j, k);
                float temp_theta = acos(dot(curr_vel, temp_vel) / length(curr_vel) / length(temp_vel));
                if(temp_id != curr_id && temp_id >= 0)
                {
                    if(temp_theta < best_theta)
                    {
                        next_id = temp_id;
                        next_vel = temp_vel;
                        best_theta = temp_theta;
                    }
                }
            }
        }
        if(best_theta < PI && (i < 20 || length(contour_points[next_id] - contour_points[begin_id]) > 2))
        {
            map_img_id(contour_points[next_id].y, contour_points[next_id].x) = -1;
            curr_id = next_id;
            curr_vel = next_vel;
        }
        else
        {
            end_id = i;
            break;
        }
    }

    for(int i = 1; i < size; i++)
    {
        // nodes(i).weight = vec2(0.5f * 540, (float((size - 1 - i) + 1) / (size + 1) * 0.5f + 0.5f * (1.f - 0.5f)) * 540);
        nodes(i).weight = (contour_points[sort_id(0.5f * end_id * i / (size - 1))] + contour_points[sort_id(end_id - 0.5f * end_id * i / (size - 1))]) / 2;
        // nodes(i).weight = vec2(float(float(i + 1) / (size + 1) * width), float(0.5 * height));
        // std::cout << node.id << ", " << node.neighbor_ids << ", " << node.weight << std::endl;
    }
}

inline int Net::SOM_Find_Min(vec2 &contour)
{
    int id = 0;
    float min_distance = 10000000000;
    for(int i = 0; i < size; i++)
    {
        float temp_distance = length(nodes(i).weight - contour);
        if(temp_distance < min_distance)
        {
            id = i;
            min_distance = temp_distance;
        }
    }
    return id;
}

inline void Net::SOM_Update(vec2 &contour, int id, float alpha, float sigma)
{
    array1d<int> map(size);

    map(id) = 1;
    nodes(id).weight = nodes(id).weight + (alpha * (contour - nodes(id).weight));

    if(sigma >= 1.0)
    {
        for(int i = 0; i < nodes(id).neighbor_ids.size; i++)
        {
            int neighbor_id = nodes(id).neighbor_ids(i);

            map(neighbor_id) = 1;
            nodes(neighbor_id).weight = nodes(neighbor_id).weight + (alpha * (nodes(id).weight - nodes(neighbor_id).weight));

            if(sigma >= 1.5)
            {
                for(int j = 0;j < nodes(neighbor_id).neighbor_ids.size;j++)
                {
                    int next_id = nodes(neighbor_id).neighbor_ids(j);

                    if(map(next_id) != 1)
                    {
                        nodes(next_id).weight = nodes(next_id).weight + (alpha * (nodes(id).weight - nodes(next_id).weight));
                    }
                }
            }
        }
    }
}

#endif