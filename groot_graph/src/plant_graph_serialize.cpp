#include <groot_graph/plant_graph_serialize.hpp>
#include <cereal/archives/binary.hpp>

namespace groot {

void write_to_file(const PlantGraph& g, std::ostream& output)
{
    cereal::BinaryOutputArchive out_archive(output);
    out_archive(g);
}

PlantGraph read_from_file(std::istream& input)
{
    PlantGraph g;
    cereal::BinaryInputArchive input_archive(input);
    input_archive(g);

    return g;
}

}