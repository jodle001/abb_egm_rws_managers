/***********************************************************************************************************************
 *
 * Tests for SystemDataParser's interpretation of a controller's option list.
 *
 ***********************************************************************************************************************
 */

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <abb_egm_rws_managers/system_data_parser.h>

using abb::robot::SystemData;
using abb::robot::SystemDataParser;

namespace
{
/**
 * \brief Builds the smallest system data the parser will accept, so that the
 *        option list is the only variable under test.
 */
SystemData makeSystemData(const std::vector<std::string>& options, const std::string& robot_ware_version)
{
  SystemData data{};

  data.ip_address = "0.0.0.0";
  data.port_number = 80;
  data.system.robot_ware_version = robot_ware_version;
  data.system.system_name = "test_system";
  data.system.system_type = "Virtual Controller";
  data.system.system_options = options;

  return data;
}

bool parsesAsMultiMove(const std::vector<std::string>& options, const std::string& robot_ware_version)
{
  SystemDataParser parser{ makeSystemData(options, robot_ware_version), "" };
  return parser.description().system_indicators().options().multimove();
}
}  // namespace

/***********************************************************
 * MultiMove detection
 *
 * The verdict decides whether parseMechanicalUnitGroups
 * keeps the controller's configured mechanical unit groups
 * or replaces them with a single synthetic group named "".
 * A false negative therefore discards real configuration
 * and puts every mechanical unit into one namespace.
 ***********************************************************/

// OmniCore numbers this option in the 3102 series. The 604 numbering is IRC5
// only, so matching the number alone missed every OmniCore MultiMove system.
TEST(SystemDataParserMultiMove, DetectsOmniCoreIndependent)
{
  EXPECT_TRUE(parsesAsMultiMove({ "MultiMove system", "3102-2 MultiMove Independent" }, "8.1.0"));
}

TEST(SystemDataParserMultiMove, DetectsOmniCoreCoordinated)
{
  EXPECT_TRUE(parsesAsMultiMove({ "MultiMove system", "3102-1 MultiMove Coordinated" }, "8.1.0"));
}

// IRC5 keeps working. These are the two strings the parser already matched,
// and they are pinned here so the OmniCore fix cannot regress them.
TEST(SystemDataParserMultiMove, DetectsIRC5Independent)
{
  EXPECT_TRUE(parsesAsMultiMove({ "604-2 MultiMove Independent" }, "6.16.3007.0"));
}

TEST(SystemDataParserMultiMove, DetectsIRC5Coordinated)
{
  EXPECT_TRUE(parsesAsMultiMove({ "604-1 MultiMove Coordinated" }, "6.16.3007.0"));
}

// A system with no MultiMove option must not be promoted. "One robot" appears
// on MultiMove and non-MultiMove systems alike, so it carries no verdict.
TEST(SystemDataParserMultiMove, SingleRobotSystemIsNotMultiMove)
{
  EXPECT_FALSE(parsesAsMultiMove({ "3124-1 Externally Guided Motion (EGM)", "One robot" }, "8.1.0"));
}

// "MultiMove system" is deliberately NOT the signal. It has only been observed
// alongside a numbered option, and what it means on its own is unverified, so
// the descriptive part of the numbered option stays the sole discriminator.
// Revisit with evidence from a controller that reports it alone.
TEST(SystemDataParserMultiMove, BareMultiMoveSystemOptionIsNotSufficient)
{
  EXPECT_FALSE(parsesAsMultiMove({ "MultiMove system" }, "8.1.0"));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
