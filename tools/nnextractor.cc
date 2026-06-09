#include <regex>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <iomanip>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>
#include <aws/s3/model/ListObjectsV2Request.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/client/ClientConfiguration.h>

#include <cereal/archives/binary.hpp>

#include "autoc/rpc/protocol.h"
#include "autoc/nn/serialization.h"
#include "autoc/util/config.h"
#include "autoc/util/s3_run_selector.h"  // 035 FR-P07 shared selector

void printUsage(const char* progName) {
  std::cout << "Usage: " << progName << " [OPTIONS]\n";
  std::cout << "Options:\n";
  std::cout << "  -k, --keyname KEYNAME    Specify run key name (autoc-timestamp)\n";
  std::cout << "  -g, --generation GEN     Specify generation number (default: latest)\n";
  std::cout << "  -o, --output FILE        Output NN weight file (default: nn_weights.dat)\n";
  std::cout << "  -i, --config FILE        Use specified config file (default: autoc.ini)\n";
  std::cout << "  -h, --help               Show this help message\n";
  std::cout << "\n";
  std::cout << "Extracts the best NN genome from an S3 evolution archive.\n";
  std::cout << "The archive contains cereal-serialized EvalResults with the NN genome\n";
  std::cout << "embedded in the .gp field (NN01 binary format).\n";
  std::cout << "\n";
  std::cout << "Examples:\n";
  std::cout << "  " << progName << "                          # Extract latest run, latest generation\n";
  std::cout << "  " << progName << " -k autoc-20250101-12345  # Extract specific run\n";
  std::cout << "  " << progName << " -g 50                    # Extract generation 50\n";
  std::cout << "  " << progName << " -o my_weights.dat        # Custom output file\n";
}

// 035 FR-P07 — gen-number parsing + run/gen selection now live in the shared
// autoc::s3_run_selector (autoc/util/s3_run_selector.h); local copy removed.

int main(int argc, char** argv) {
  static struct option long_options[] = {
    {"keyname", required_argument, 0, 'k'},
    {"generation", required_argument, 0, 'g'},
    {"output", required_argument, 0, 'o'},
    {"config", required_argument, 0, 'i'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };

  std::string computedKeyName = "";
  int specifiedGeneration = -1;
  std::string outputFile = "nn_weights.dat";
  std::string configFile = "autoc.ini";
  int option_index = 0;
  int c;

  while ((c = getopt_long(argc, argv, "k:g:o:i:h", long_options, &option_index)) != -1) {
    switch (c) {
      case 'k': computedKeyName = optarg; break;
      case 'g': specifiedGeneration = std::stoi(optarg); break;
      case 'o': outputFile = optarg; break;
      case 'i': configFile = optarg; break;
      case 'h': printUsage(argv[0]); return 0;
      case '?': printUsage(argv[0]); return 1;
      default: break;
    }
  }

  // Positional argument for backward compat
  if (computedKeyName.empty() && optind < argc) {
    computedKeyName = argv[optind];
  }

  // Initialize configuration and AWS
  ConfigManager::initialize(configFile);

  Aws::SDKOptions options;
  Aws::InitAPI(options);

  auto s3_client = ConfigManager::getS3Client();
  std::string bucket = ConfigManager::getConfig().s3Bucket;

  // 035 FR-P07 — run/gen selection via the shared selector (fail-loud).
  // -k may be given without a trailing slash; normalize so gen lookup works.
  std::string keyName;
  try {
    if (!computedKeyName.empty() && computedKeyName.back() != '/') computedKeyName += '/';
    if (computedKeyName.empty()) {
      computedKeyName = autoc::findLatestRun(*s3_client, bucket);
    }
    if (specifiedGeneration >= 0) {
      // -g passes the raw file gen number (10000 - actualGen), as before.
      keyName = computedKeyName + "gen" + std::to_string(specifiedGeneration) + ".dmp";
    } else {
      keyName = autoc::findLatestGenKey(*s3_client, bucket, computedKeyName);
    }
  } catch (const std::exception& e) {
    std::cerr << "Run-selector error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Fetching: " << keyName << " from " << bucket << std::endl;

  // Fetch + inflate via the shared S3 dmp I/O (035 FR-P09), then deserialize.
  EvalResults evalResults;
  try {
    std::string retrievedData = autoc::s3GetDmpBlob(*s3_client, bucket, keyName);
    std::istringstream iss(retrievedData, std::ios::binary);
    cereal::BinaryInputArchive ia(iss);
    ia(evalResults);
  }
  catch (const std::exception& e) {
    std::cerr << "Error fetching/deserializing " << keyName << ": " << e.what() << std::endl;
    return 1;
  }

  // Extract NN genome from .gp field
  if (evalResults.gp.empty()) {
    std::cerr << "No controller data found in EvalResults" << std::endl;
    return 1;
  }

  // Verify it's NN format
  if (!nn_detect_format(reinterpret_cast<const uint8_t*>(evalResults.gp.data()),
                        evalResults.gp.size())) {
    std::cerr << "Error: S3 archive does not contain NN data (no NN01 magic bytes)." << std::endl;
    return 1;
  }

  // Deserialize to get metadata
  NNGenome genome;
  if (!nn_deserialize(reinterpret_cast<const uint8_t*>(evalResults.gp.data()),
                      evalResults.gp.size(), genome)) {
    std::cerr << "Error deserializing NN genome from archive" << std::endl;
    return 1;
  }

  // Set provenance: profile:bucket/key
  std::string profile = ConfigManager::getConfig().s3Profile;
  genome.source = profile + ":" + bucket + "/" + keyName;
  std::vector<uint8_t> nnData;
  nn_serialize(genome, nnData);

  std::ofstream file(outputFile, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: Cannot create output file: " << outputFile << std::endl;
    return 1;
  }
  file.write(reinterpret_cast<const char*>(nnData.data()), nnData.size());
  file.close();

  // Print summary
  int generation = autoc::extractGenNumber(keyName);
  std::cout << "\nExtracted NN genome to: " << outputFile << std::endl;
  std::cout << "  S3 Key:    " << keyName << std::endl;
  std::cout << "  Generation: " << generation << std::endl;
  std::cout << "  Topology:   ";
  for (size_t i = 0; i < genome.topology.size(); i++) {
    if (i > 0) std::cout << " -> ";
    std::cout << genome.topology[i];
  }
  std::cout << std::endl;
  std::cout << "  Weights:    " << genome.weights.size() << std::endl;
  std::cout << "  Fitness:    " << std::fixed << std::setprecision(6) << genome.fitness << std::endl;
  std::cout << "  Sigma:      " << std::fixed << std::setprecision(6) << genome.mutation_sigma << std::endl;
  std::cout << "  File size:  " << evalResults.gp.size() << " bytes" << std::endl;

  Aws::ShutdownAPI(options);
  return 0;
}
