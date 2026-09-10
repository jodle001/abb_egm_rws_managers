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
 * \brief Builds system data with the general system info and one mechanical
 *        unit group filled in, so the option list is the variable under test.
 *        The parser tolerates the other configuration lists being empty; the
 *        manager is what insists on them. The group is there because a
 *        MultiMove verdict with no groups is an error, which has its own test.
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

  abb::rws::cfg::sys::MechanicalUnitGroup group{};
  group.name = "rob1";
  group.robot = "ROB_1";
  data.configurations.mechanical_unit_groups = { group };

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
// 3102-2 has been read from a controller; 3102-1 below is inferred from the
// IRC5 pair and has not. The matcher ignores the number, so the case pins the
// description, not the numbering.
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

// The description is matched as the option's suffix: text that merely
// contains it does not count.
TEST(SystemDataParserMultiMove, DescriptionMustEndTheOption)
{
  EXPECT_FALSE(parsesAsMultiMove({ "3102-2 MultiMove Independent (removed)" }, "8.1.0"));
}

/***********************************************************
 * Mechanical unit groups
 *
 * What the verdict is for: a MultiMove system keeps the
 * controller's configured groups by name, anything else is
 * folded into one synthetic group named "".
 ***********************************************************/

namespace
{
SystemData makeGroupedSystemData(const std::vector<std::string>& options)
{
  SystemData data{ makeSystemData(options, "8.1.0") };

  abb::rws::cfg::sys::MechanicalUnitGroup arm{};
  arm.name = "rob1";
  arm.robot = "ROB_1";

  abb::rws::cfg::sys::MechanicalUnitGroup positioner{};
  positioner.name = "extax";
  positioner.mechanical_units = { "MU_1" };

  data.configurations.mechanical_unit_groups = { arm, positioner };
  return data;
}
}  // namespace

TEST(SystemDataParserGroups, MultiMoveKeepsTheConfiguredGroups)
{
  SystemDataParser parser{ makeGroupedSystemData({ "3102-2 MultiMove Independent" }), "" };
  const auto description{ parser.description() };
  const auto& groups{ description.mechanical_units_groups() };

  ASSERT_EQ(groups.size(), 2);
  EXPECT_EQ(groups.Get(0).name(), "rob1");
  EXPECT_EQ(groups.Get(1).name(), "extax");
}

TEST(SystemDataParserGroups, NonMultiMoveFoldsEverythingIntoOneSyntheticGroup)
{
  SystemDataParser parser{ makeGroupedSystemData({ "One robot" }), "" };
  const auto description{ parser.description() };
  const auto& groups{ description.mechanical_units_groups() };

  ASSERT_EQ(groups.size(), 1);
  EXPECT_EQ(groups.Get(0).name(), "");
}

TEST(SystemDataParserGroups, MultiMoveWithoutGroupsIsAnError)
{
  SystemData data{ makeSystemData({ "604-2 MultiMove Independent" }, "6.16.3007.0") };
  data.configurations.mechanical_unit_groups.clear();

  EXPECT_THROW(SystemDataParser(data, ""), std::runtime_error);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
