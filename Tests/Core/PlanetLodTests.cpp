#include "PlanetLodContractChecks.h"
#include <iostream>
int main()
{
    PlanetTests::LodCheckReport Report;
    if (!PlanetTests::RunLodContractChecks(Report))
    {
        std::cerr << "FAILED: " << Report.Failure << '\n';
        return 1;
    }
    std::cout << "{\"status\":\"passed\",\"checks\":" << Report.Checks
        << ",\"stitch_masks\":8,\"root_orientations\":20,\"deep_level\":32,\"mixed_edits\":500,\"runtime_streaming_validated\":false}\n";
}
