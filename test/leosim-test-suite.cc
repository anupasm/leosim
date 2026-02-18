/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ns3/leosim.h"
#include "ns3/test.h"

using namespace ns3;

/**
 * \ingroup leosim-tests
 * \defgroup leosim-test-suite leosim module tests
 */

/**
 * \ingroup leosim-test-suite
 * \brief LeoSim basic test case
 */
class LeoSimTestCase1 : public TestCase
{
  public:
    LeoSimTestCase1();
    ~LeoSimTestCase1() override;

  private:
    void DoRun() override;
};

LeoSimTestCase1::LeoSimTestCase1()
    : TestCase("LeoSim test case (does nothing)")
{
}

LeoSimTestCase1::~LeoSimTestCase1()
{
}

void
LeoSimTestCase1::DoRun()
{
    // Test instantiation
    Ptr<LeoSim> leosim = CreateObject<LeoSim>();
    NS_TEST_ASSERT_MSG_NE(leosim, nullptr, "Failed to create LeoSim object");
}

/**
 * \ingroup leosim-test-suite
 * \brief LeoSim test suite
 */
class LeoSimTestSuite : public TestSuite
{
  public:
    LeoSimTestSuite();
};

LeoSimTestSuite::LeoSimTestSuite()
    : TestSuite("leosim", Type::UNIT)
{
    AddTestCase(new LeoSimTestCase1, TestCase::Duration::QUICK);
}

static LeoSimTestSuite sLeoSimTestSuite; //!< Static variable for test initialization
