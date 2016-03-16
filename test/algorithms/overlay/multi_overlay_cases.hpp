// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2010-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_TEST_MULTI_OVERLAY_CASES_HPP
#define BOOST_GEOMETRY_TEST_MULTI_OVERLAY_CASES_HPP


#include <string>

// See powerpoint doc/other/test_cases/overlay_cases.ppt
// Note that there are some duplicates with single (80 and on)

static std::string case_multi_simplex[2] =
{
    "MULTIPOLYGON(((0 1,2 5,5 3,0 1)),((1 1,5 2,5 0,1 1)))",
    "MULTIPOLYGON(((3 0,0 3,4 5,3 0)))"
};

// To mix multi/single
static std::string case_single_simplex = "POLYGON((3 0,0 3,4 5,3 0))";

static std::string case_multi_no_ip[2] =
{
    "MULTIPOLYGON(((4 1,0 7,7 9,4 1)),((8 1,6 3,10 4,8 1)),((12 6,10 7,13 8,12 6)))",
    "MULTIPOLYGON(((14 4,8 8,15 10,14 4)),((15 3,18 9,20 2,15 3)),((3 4,1 7,5 7,3 4)))"
};

static std::string case_multi_2[2] =
{
    "MULTIPOLYGON(((4 3,2 7,10 9,4 3)),((8 1,6 3,10 4,8 1)),((12 6,10 7,13 8,12 6)))",
    "MULTIPOLYGON(((14 4,8 8,15 10,14 4)),((15 3,18 9,20 2,15 3)),((5 5,4 7,7 7,5 5)))"
};

// Case 58, same as case_58 IET (single) but here the second polygon is inverted
// To check behaviour of difference, but in an intersection operation
static std::string case_58_multi[8] =
{
    /* a */ "MULTIPOLYGON(((3 3,3 4,4 4,4 3,3 3)))",
    /* b */ "MULTIPOLYGON(((0 2,0 5,4 4,5 0,0 2),(4 4,1 4,1 3,4 4),(4 4,2 3,2 2,4 4),(4 4,3 2,4 2,4 4)))",
    /* a inv */ "MULTIPOLYGON(((-1 -1,-1 6,6 6,6 -1,-1 -1),(3 3,4 3,4 4,3 4,3 3)))",
    /* b inv */ "MULTIPOLYGON(((6 6,6 0,5 0,4 4,0 5,0 6,6 6)),((4 4,1 3,1 4,4 4)),((4 4,2 2,2 3,4 4)),((4 4,4 2,3 2,4 4)))",

    // simpler versions of b
    /* b */ "MULTIPOLYGON(((0 2,0 5,4 4,5 0,0 2),(4 4,1 4,1 3,4 4)))",
    /* b */ "MULTIPOLYGON(((0 2,0 5,4 4,5 0,0 2),(4 4,3 2,4 2,4 4)))",
    /* b */ "MULTIPOLYGON(((0 2,0 5,4 4,5 0,0 2),(4 4,2 3,2 2,4 4)))",
    /* b */ "MULTIPOLYGON(((0 2,0 5,4 4,5 0,0 2),(4 4,2 3,2 2,4 4),(4 4,3 2,4 2,4 4)))",
};

static std::string case_61_multi[2] =
{
    // extracted from recursive boxes
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((2 2,2 3,3 3,3 2,2 2)))",
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)),((2 3,2 4,3 4,3 3,2 3)))"
};

static std::string case_62_multi[2] =
{
    // extracted from recursive boxes
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)))",
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)),((2 3,2 4,3 4,3 3,2 3)))"
};

static std::string case_63_multi[2] =
{
    // extracted from recursive boxes
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)))",
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)),((2 1,2 2,3 2,3 1,2 1)))"
};

static std::string case_64_multi[3] =
{
    // extracted from recursive boxes
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((2 2,2 3,3 3,3 2,2 2)))",
    "MULTIPOLYGON(((1 1,1 2,2 2,3 2,3 1,2 1,1 1)))" ,
    // same but omitting not-necessary form-points at x=2 (==simplified)
    "MULTIPOLYGON(((1 1,1 2,3 2,3 1,1 1)))"
};

static std::string case_65_multi[4] =
{
    "MULTIPOLYGON(((2 2,2 3,3 3,3 2,2 2)))",
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((2 2,2 3,3 3,3 2,2 2)),((3 1,3 2,5 2,5 1,3 1)))",

    // Inverse versions
    "MULTIPOLYGON(((0 0,0 4,6 4,6 0,0 0),(2 2,3 2,3 3,2 3,2 2)))",
    "MULTIPOLYGON(((0 0,0 4,6 4,6 0,0 0),(1 1,2 1,2 2,1 2,1 1),(2 2,3 2,3 3,2 3,2 2),(3 1,5 1,5 2,3 2,3 1)))"
};

static std::string case_66_multi[2] =
{
    "MULTIPOLYGON(((3 5,2 5,2 6,3 6,4 6,4 5,3 5)),((1 6,0 6,0 7,1 7,2 7,2 6,1 6)))",
    "MULTIPOLYGON(((1 4,1 5,2 5,2 4,1 4)),((1 7,2 7,2 6,1 6,1 7)),((0 8,0 9,1 9,1 8,1 7,0 7,0 8)))"
};

static std::string case_67_multi[2] =
{
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)),((2 1,2 2,3 2,3 1,2 1)))",
    "MULTIPOLYGON(((1 1,1 2,3 2,3 1,1 1)))"
};

static std::string case_68_multi[2] =
{
    "MULTIPOLYGON(((2 1,2 2,4 2,4 1,2 1)),((4 2,4 3,5 3,5 2,4 2)))",
    "MULTIPOLYGON(((1 2,1 3,2 3,2 2,1 2)),((2 1,2 2,3 2,3 1,2 1)),((3 2,3 3,5 3,5 2,3 2)))"
};

static std::string case_69_multi[2] =
{
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((3 2,3 3,4 3,4 2,3 2)))",
    "MULTIPOLYGON(((2 0,2 1,3 1,3 0,2 0)),((1 1,1 3,2 3,2 1,1 1)),((2 3,2 4,3 4,3 3,2 3)))"
};

static std::string case_71_multi[2] =
{
    "MULTIPOLYGON(((0 0,0 3,1 3,1 1,3 1,3 2,4 2,4 0,0 0)),((2 2,2 3,3 3,3 2,2 2)))",
    "MULTIPOLYGON(((0 2,0 3,3 3,3 2,0 2)))"
};

static std::string case_72_multi[3] =
{
    // cluster with ii, done by both traverse and assemble
    "MULTIPOLYGON(((0 3,4 4,3 0,3 3,0 3)),((3 3,2 1,1 2,3 3)))",
    "MULTIPOLYGON(((0 0,1 4,3 3,4 1,0 0)))",

    // Inverse version of a
    "MULTIPOLYGON(((-1 -1,-1 5,5 5,5 -1,-1 -1),(0 3,3 3,3 0,4 4,0 3),(3 3,1 2,2 1,3 3)))"
};

static std::string case_73_multi[2] =
{
    "MULTIPOLYGON(((2 2,2 3,3 3,3 2,2 2)),((1 1,1 2,2 2,2 1,1 1)))",
    "MULTIPOLYGON(((1 1,1 2,2 2,2 3,3 3,3 1,1 1)))"
};

static std::string case_74_multi[2] =
{
    "MULTIPOLYGON(((3 0,2 0,2 1,3 1,3 3,1 3,1 2,2 2,2 1,0 1,0 5,4 5,4 0,3 0)))",
    "MULTIPOLYGON(((0 2,0 3,1 3,1 1,2 1,2 0,0 0,0 2)),((2 3,1 3,1 4,2 4,2 3)))"
};

static std::string case_75_multi[2] =
{
    // cc/uu turns on all corners of second box
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((1 3,1 4,2 4,2 3,1 3)),((2 2,2 3,3 3,3 2,2 2)),((3 1,3 2,4 2,4 1,3 1)),((3 3,3 4,4 4,4 3,3 3)))",
    "MULTIPOLYGON(((2 2,2 3,3 3,3 2,2 2)))"
};

static std::string case_76_multi[2] =
{
    // cc/uu turns on all corners of second box, might generate TWO OVERLAPPING union polygons!
    // therefore, don't follow uu.
    "MULTIPOLYGON(((1 0,1 1,2 1,2 0,1 0)),((3 2,4 2,4 1,3 1,3 2)),((2 2,2 3,3 3,3 2,2 2)),((2 3,1 3,1 4,2 4,2 3)),((3 3,3 4,4 4,4 3,3 3)))",
    "MULTIPOLYGON(((0 2,0 3,1 3,1 2,2 2,2 0,1 0,1 1,0 1,0 2)),((2 2,2 3,3 3,3 2,2 2)))"
};

static std::string case_77_multi[2] =
{
    // with a point on interior-ring-border of enclosing
    // -> went wrong in the assemble phase for intersection (traversal is OK)
    // -> fixed
    "MULTIPOLYGON(((3 3,3 4,4 4,4 3,3 3)),((5 3,5 4,4 4,4 5,3 5,3 6,5 6,5 5,7 5,7 6,8 6,8 5,9 5,9 2,8 2,8 1,7 1,7 2,5 2,5 3),(6 3,8 3,8 4,6 4,6 3)))",
    "MULTIPOLYGON(((6 3,6 4,7 4,7 3,6 3)),((2 3,1 3,1 4,3 4,3 5,4 5,4 6,5 6,5 7,9 7,9 4,7 4,7 5,8 5,8 6,7 6,7 5,6 5,6 4,4 4,4 3,3 3,3 2,2 2,2 3)),((5 2,4 2,4 3,6 3,6 2,5 2)),((7 2,7 3,8 3,8 2,8 1,7 1,7 2)))"
};

static std::string case_78_multi[2] =
{
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(2 2,4 2,4 3,2 3,2 2)))",
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(3 2,4 2,4 3,3 3,3 2),(1 1,2 1,2 2,1 2,1 1)))"

};

static std::string case_80_multi[2] =
{
    // Many ux-clusters -> needs correct cluster-sorting
    // Fixed now
    "MULTIPOLYGON(((3 1,3 2,4 2,3 1)),((1 5,0 4,0 5,1 6,1 5)),((3 3,4 3,3 2,2 2,2 3,3 3)),((4 5,5 6,5 5,4 5)),((4 2,4 3,5 3,4 2)),((2.5 5.5,3 5,2 5,2 7,3 6,2.5 5.5)),((1 6,0 6,0 7,1 7,2 6,1 6)))",
    "MULTIPOLYGON(((3 5,3 6,4 6,4 5,3 5)),((4 4,5 5,5 4,4 4)),((3 3,4 4,4 3,3 3)),((1 5,1 6,2 6,2 5,1 5)),((0 6,1 7,1 6,0 6)),((1 4,1 3,0 3,0 4,1 4)),((3 5,4 4,3 4,3 3,2 3,2 5,3 5)))"
};

static std::string case_81_multi[2] =
{
    "MULTIPOLYGON(((1 1,2 2,2 1,1 1)),((2 2,2 3,3 2,2 2)),((3 1,4 2,4 1,3 1)))",
    "MULTIPOLYGON(((2 1,2 2,3 3,3 2,4 2,3 1,2 1)))"
};

static std::string case_82_multi[2] =
{
    "MULTIPOLYGON(((4 0,5 1,5 0,4 0)),((2 1,3 2,3 1,2 1)),((3 0,4 1,4 0,3 0)),((1 0,1 1,2 1,2 0,1 0)))",
    "MULTIPOLYGON(((3 2,4 3,4 2,3 2)),((3 1,3 2,4 1,3 1)),((0 0,1 1,1 0,0 0)),((5 1,5 0,4 0,4 1,5 1)))"
};

static std::string case_83_multi[2] =
{
    // iu/iu
    "MULTIPOLYGON(((1 0,1 1,2 1,1 0)),((0 1,0 4,1 4,1 1,0 1)),((2 1,2 2,3 2,3 1,2 1)),((2 3,3 4,3 3,2 3)))",
    "MULTIPOLYGON(((1 0,2 1,2 0,1 0)),((0 3,1 4,1 3,0 3)),((2 3,2 4,3 3,2 3)),((1 3,2 3,2 2,0 2,1 3)))"
};

static std::string case_84_multi[2] =
{
    // iu/ux
    "MULTIPOLYGON(((2 2,3 3,3 2,2 2)),((2 1,2 2,3 1,2 1)),((2 3,3 4,3 3,2 3)),((1 3,2 4,2 2,1 2,1 3)))",
    "MULTIPOLYGON(((2 3,3 3,3 1,2 1,2 2,1 2,1 3,2 3)))"
};

static std::string case_85_multi[2] =
{
    // iu/ux (and ux/ux)
    "MULTIPOLYGON(((0 1,0 2,1 2,0 1)),((1 1,1 2,2 1,1 1)),((0 3,1 3,0 2,0 3)))",
    "MULTIPOLYGON(((1 3,2 3,2 1,1 1,1 2,0 2,1 3)))"
};

static std::string case_86_multi[2] =
{
    // iu/ux
    "MULTIPOLYGON(((4 2,4 3,5 3,4 2)),((5 2,6 3,6 2,5 2)),((5 1,4 1,4 2,5 2,6 1,5 1)))",
    "MULTIPOLYGON(((5 1,5 2,6 2,6 1,5 1)),((4 2,5 3,5 2,4 2)),((3 2,4 3,4 2,3 2)))"
};

static std::string case_87_multi[2] =
{
    // iu/ux where iu crosses, no touch
    "MULTIPOLYGON(((5 0,5 1,6 0,5 0)),((6 2,7 3,7 2,6 2)),((5 1,5 3,6 3,6 1,5 1)))",
    "MULTIPOLYGON(((5 1,5 2,7 2,7 1,6 1,6 0,5 0,5 1)),((4 3,5 3,5 2,3 2,4 3)))"
};


static std::string case_88_multi[2] =
{
    "MULTIPOLYGON(((0 0,0 1,1 0,0 0)),((1 1,1 2,2 1,1 1)),((0 2,0 3,1 3,2 3,2 2,1 2,0 1,0 2)))",
    "MULTIPOLYGON(((0 0,0 1,1 0,0 0)),((0 1,1 2,1 1,0 1)),((0 2,0 3,1 3,1 2,0 2)))"
};

static std::string case_89_multi[2] =
{
    // Extract from rec.boxes_3
    "MULTIPOLYGON(((8 1,7 1,8 2,8 3,9 4,9 2,8.5 1.5,9 1,8 0,8 1)),((9 1,9 2,10 2,10 1,9 0,9 1)))",
    "MULTIPOLYGON(((8 3,9 4,9 3,8 3)),((7 0,7 1,8 1,8 0,7 0)),((9 2,9 1,8 1,8 3,8.5 2.5,9 3,9 2)))"
};

static std::string case_90_multi[2] =
{
    // iu/iu for Union; see ppt
    "MULTIPOLYGON(((1 8,0 8,0 10,1 10,1 9,2 8,2 7,1 7,1 8)),((2 9,2 10,4 10,4 9,3 9,3 8,2 8,2 9)))",
    "MULTIPOLYGON(((2 8,1 8,1 9,2 9,2 10,3 10,3 8,2 8)),((0 10,2 10,0 8,0 10)))"
};

static std::string case_91_multi[2] =
{
    // iu/xi for Intersection
    "MULTIPOLYGON(((3 3,3 4,4 4,3 3)),((2 2,1 2,1 4,2 4,2 3,3 3,2 2)))",
    "MULTIPOLYGON(((2 2,2 3,3 2,2 2)),((2 3,1 3,1 4,1.5 3.5,2 4,2.5 3.5,3 4,3 3,2 3)))"
};

static std::string case_92_multi[2] =
{
    // iu/iu all aligned (for union)
    "MULTIPOLYGON(((7 2,7 3,8 2,7 2)),((8 4,9 5,9 4,8 4)),((8 2,8 3,9 2,8 2)),((7 3,7 4,8 4,8 3,7 3)),((9 3,9 4,10 4,10 3,9 3)))",
    "MULTIPOLYGON(((9 2,8 2,8 3,9 3,10 2,9 2)),((7 5,8 5,9 6,9 4,8 4,7 3,6 3,6 4,6.5 3.5,7 4,6 4,7 5)))"
};

static std::string case_93_multi[2] =
{
    // iu/xi for intersection
    "MULTIPOLYGON(((6 2,7 2,7 1,5 1,6 2)),((7 3,8 3,7.5 2.5,8 2,7 2,7 3)))",
    "MULTIPOLYGON(((7 1,6 0,6 2,7 3,7 2,8 3,8 2,7 1)))"
};


static std::string case_94_multi[2] =
{
    // iu/iu for union
    "MULTIPOLYGON(((9 2,9 3,10 3,10 2,9 2)),((7 3,8 4,9 3,8 3,9 2,7 2,7 3)),((8 6,9 5,9 4,8 4,8 6)))",
    "MULTIPOLYGON(((6 2,6 3,7 3,8 2,6 2)),((9 3,10 3,9 2,9 1,8 0,7 0,8 1,8 3,8.5 2.5,9 3)),((7 4,7 5,8 5,9 6,9 4,8 4,8 3,7 3,7 4)))"
};

static std::string case_95_multi[2] =
{
    // iu/iu for union
    "MULTIPOLYGON(((0 8,1 8,1 7,0 7,0 8)),((2 8,2 9,2.5 8.5,3 9,3 7,2 7,2 8)),((1 9,1 10,2 9,1 8,1 9)))",
    "MULTIPOLYGON(((1 7,0 7,0 8,1 8,2 7,1 7)),((2 9,1 9,1 10,2 10,3 9,4 9,4 8,2 8,2 9)))"
};

static std::string case_96_multi[2] =
{
    // iu/iu all collinear, for intersection/union
    "MULTIPOLYGON(((8 2,9 3,9 2,8 2)),((8 1,9 2,9 1,10 1,10 0,8 0,8 1)))",
    "MULTIPOLYGON(((9 0,9 1,10 0,9 0)),((8 1,8 2,9 2,9 1,8 1)))"
};

static std::string case_97_multi[2] =
{
    // ux/ux for union
    "MULTIPOLYGON(((4 4,4 5,4.5 4.5,5 5,6 5,5 4,5 3,4 3,4 4)))",
    "MULTIPOLYGON(((5 3,5 4,6 3,5 3)),((6 5,7 5,6 4,5 4,6 5)))"
};


static std::string case_98_multi[2] =
{
    // ii/iu for intersection, solved by discarding iu (ordering not possible)
    "MULTIPOLYGON(((2 0,3 1,3 0,2 0)),((2 2,2 3,1 3,1 4,2 4,3 3,3 4,5 2,4 2,4 1,3 1,3 2,2.5 1.5,3 1,2 1,2 2)))",
    "MULTIPOLYGON(((4 2,4 3,5 2,4 2)),((1 0,0 0,0 2,4 2,4 1,2 1,2 0,1 0)),((3 3,4 4,4 3,3 2,3 3)))"
};

static std::string case_99_multi[2] =
{
    // iu/iu for intersection
    "MULTIPOLYGON(((1 0,2 1,2 0,1 0)),((1 2,2 2,1.5 1.5,2 1,1 1,1 0,0 0,0 1,1 2)))",
    "MULTIPOLYGON(((1 1,2 0,0 0,1 1)),((1 1,0 1,0 2,1 2,2 3,2 2,1 1)))"
};

static std::string case_100_multi[2] =
{
    // for intersection
    "MULTIPOLYGON(((0 0,0 1,1 0,0 0)),((2 2,2 1,0 1,0 2,1 2,2 3,2 2)))",
    "MULTIPOLYGON(((1 1,1 2,2 2,2 1,1 1)),((1 2,0 1,0 3,1 4,1 2)))"
};

static std::string case_101_multi[2] =
{
    // interior ring / union
    "MULTIPOLYGON(((7 2,7 3,8 2,7 2)),((9 3,9 4,10 3,9 3)),((10 1,10 0,8 0,8 1,9 2,10 2,10 1)),((9 3,9 2,8 2,8 3,7 3,7 4,8 4,9 3)),((8 4,8 7,9 6,9 4,8 4)))",
    "MULTIPOLYGON(((6 1,5 1,5 2,6 3,6 4,7 5,6 5,7 6,8 6,8 5,9 5,8 4,9 4,9 5,10 5,10 1,8 1,8 3,7 3,7 2,6 2,7 1,8 1,7 0,5 0,5 1,5.5 0.5,6 1),(8.5 2.5,9 2,9 3,8.5 2.5)))"
};

static std::string case_102_multi[4] =
{
    // interior ring 'fit' / union
    "MULTIPOLYGON(((0 2,0 7,5 7,5 2,0 2),(4 3,4 6,1 6,2 5,1 5,1 4,3 4,4 3)),((3 4,3 5,4 5,3 4)),((2 5,3 6,3 5,2 4,2 5)))",
    "MULTIPOLYGON(((0 2,0 7,5 7,5 2,0 2),(2 4,3 5,2 5,2 4),(4 4,3 4,3 3,4 4),(4 5,4 6,3 6,4 5)))",

    /* inverse versions (first was already having an interior, so outer ring is just removed */
    "MULTIPOLYGON(((4 3,3 4,1 4,1 5,2 5,1 6,4 6,4 3),(3 4,4 5,3 5,3 4),(2 5,2 4,3 5,3 6,2 5)))",
    "MULTIPOLYGON(((-1 1,-1 8,6 8,6 1,-1 1),(0 2,5 2,5 7,0 7,0 2)),((2 4,2 5,3 5,2 4)),((4 4,3 3,3 4,4 4)),((4 5,3 6,4 6,4 5)))"
};

static std::string case_103_multi[2] =
{
    // interior ring 'fit' (ix) / union / assemble
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,2 0,2 1,3 1,3 2,2 2,2 3,1 2,2 2,2 1,1 0,0 0)))",
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(2 1,2 2,1 1,2 1)))"
};

static std::string case_104_multi[2] =
{
    // interior ring 'fit' (ii) / union / assemble
    "MULTIPOLYGON(((1 0,1 1,0 1,0 5,5 5,5 0,2 0,2 1,1 0),(2 2,3 3,2 3,2 2)))",
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(1 1,3 1,3 2,1 2,1 1)))"
};

static std::string case_105_multi[2] =
{
    // interior ring 'fit' () / union / assemble
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(2 2,3 2,3 3,1 3,2 2)))",
    "MULTIPOLYGON(((0 0,0 5,5 5,5 0,0 0),(1 1,2 1,2 2,1 1),(2 1,3 1,3 2,2 1),(1 3,3 3,3 4,2 3,2 4,1 4,1 3)))"
};

static std::string case_106_multi[2] =
{
    // interior ring 'fit' () / union / assemble
    "MULTIPOLYGON(((0 0,0 3,1 2,1 3,2 3,2 1,3 2,2 2,3 3,2 3,3 4,1 4,1 3,0 3,0 5,5 5,5 0,0 0)))",
    "MULTIPOLYGON(((0 0,0 5,1 5,1 4,2 4,2 5,3 5,3 3,4 4,5 4,5 0,2 0,3 1,2 1,2 3,1 3,2 2,1.5 1.5,2 1,1 1,0 0)),((1 0,2 1,2 0,1 0)))"
};

static std::string case_107_multi[4] =
{
    // For CCW polygon reports a iu/iu problem.
    "MULTIPOLYGON(((6 8,7 9,7 7,8 7,7 6,6 6,6 8)),((6.5 9.5,7 10,7 9,6 9,6 10,6.5 9.5)))",
    "MULTIPOLYGON(((5 7,6 8,6 10,7 9,8 10,8 8,7 8,6 7,6 6,5 7)))",

    /* inverse versions */
    "MULTIPOLYGON(((5 5,5 11,9 11,9 5,5 5),(6 8,6 6,7 6,8 7,7 7,7 9,6 8),(6.5 9.5,6 10,6 9,7 9,7 10,6.5 9.5)))",
    "MULTIPOLYGON(((4 5,4 11,9 11,9 5,4 5),(5 7,6 6,6 7,7 8,8 8,8 10,7 9,6 10,6 8,5 7)))"
};

static std::string case_108_multi[3] =
{
    // Missing intersection point in [0] / [1], [0] / [2] is OK
    "MULTIPOLYGON(((3 4,4 4,4 2,4 1,1 1,1 2,2 2,2.5 1.5,3 2,2 2,3 3,1 3,1 2,0 1,0 6,1 6,2 5,2 4,1 4,1.5 3.5,2 4,3 4)))",
    "MULTIPOLYGON(((0 3,0 4,1 4,0 3)),((3 6,3 5,5 5,5 0,4 0,4 1,1 1,1 2,2 3,2 4,2.5 3.5,3 4,4 3,3 3,3 2,4 2,4 4,2 4,1 3,1 5,2 6,3 6)),((1 3,1 2,0 2,0 3,1 3)))",
    "MULTIPOLYGON(((0 3,0 4,1 4,0 3)),((3 6,3 5,5 5,5 1,        1 1,1 2,2 3,2 4,2.5 3.5,3 4,4 3,3 3,3 2,4 2,4 4,2 4,1 3,1 5,2 6,3 6)),((1 3,1 2,0 2,0 3,1 3)))"
};

static std::string case_recursive_boxes_1[2] =
{
    // == 70
    // Used in blog. KEEP
    "MULTIPOLYGON(((1 0,0 0,0 1,1 1,1 2,0 2,0 4,2 4,2 5,3 5,3 6,1 6,1 5,0 5,0 10,9 10,9 9,7 9,7 8,6 8,6 7,8 7,8 6,9 6,9 4,8 4,8 5,7 5,7 6,6 6,6 5,5 5,5 4,4 4,4 3,5 3,5 2,7 2,7 3,6 3,6 4,8 4,8 3,10 3,10 0,6 0,6 1,5 1,5 0,1 0),(4 7,4 9,3 9,3 7,4 7),(8 1,9 1,9 2,8 2,8 1)),((10 7,10 6,9 6,9 7,8 7,8 8,9 8,9 9,10 9,10 7)))",
    "MULTIPOLYGON(((1 0,0 0,0 7,2 7,2 6,5 6,5 5,4 5,4 4,5 4,5 3,7 3,7 2,6 2,6 0,1 0),(2 1,2 2,3 2,3 3,1 3,1 1,2 1)),((7 0,7 2,10 2,10 0,9 0,9 1,8 1,8 0,7 0)),((6 4,6 6,5 6,5 7,6 7,6 8,5 8,5 7,3 7,3 9,2 9,2 8,1 8,1 10,4 10,4 9,6 9,6 10,10 10,10 9,9 9,9 8,10 8,10 6,9 6,9 5,10 5,10 3,7 3,7 4,6 4),(7 7,7 6,8 6,8 7,7 7)))"
};

static std::string case_recursive_boxes_2[2] =
{
    // Traversal problem; Many ii-cases -> formerly caused "Endless loop"
    // So it appears that there might be more decisions than intersection points
    "MULTIPOLYGON(((1 0,0 0,0 4,1 4,1 5,0 5,0 10,3 10,3 9,4 9,4 10,6 10,6 9,5 9,5 8,6 8,6 9,7 9,7 10,10 10,10 0,1 0),(7 6,8 6,8 7,9 7,9 8,8 8,8 7,7 7,7 6),(9 1,9 2,8 2,8 1,9 1)))",
    "MULTIPOLYGON(((0 0,0 10,10 10,10 0,8 0,8 1,7 1,7 0,0 0),(7 3,6 3,6 2,7 2,7 3),(6 7,7 7,7 8,6 8,6 7)))"
};


static std::string case_recursive_boxes_3[2] =
{
    // Previously a iu/ux problem causing union to fail.
    // For CCW polygon it also reports a iu/iu problem.
    // KEEP
    "MULTIPOLYGON(((8 3,9 4,9 3,8 3)),((5 9,5 10,6 10,5 9)),((2 0,2 1,3 0,2 0)),((2 5,2 6,3 6,3 5,2 5)),((2 2,1 2,1 3,2 3,3 2,3 1,2 1,2 2)),((6 8,7 9,7 7,8 7,7 6,6 6,6 8)),((4 6,5 7,5 6,4 6)),((4 8,4 9,5 9,5 8,4 8)),((0 3,1 4,1 3,0 3)),((8 7,9 8,9 7,8 7)),((9 6,9 7,10 7,9 6)),((7 0,8 1,8 0,7 0)),((0 4,0 5,1 5,1 4,0 4)),((4 2,5 3,5 2,4 1,4 2)),((4 10,4 9,2 9,3 10,4 10)),((5 2,6 3,7 3,7 2,6 2,6 1,5 0,5 2)),((5 3,4 3,4 4,2 4,4 6,4 5,4.5 4.5,6 6,6 5,7 4,5 4,5 3)),((10 2,9 1,9 3,10 2)),((8 4,7 4,8 5,7 5,7 6,9 6,9 5,10 5,10 4,8 4)),((1 7,0 7,0 8,1 8,1 7)),((1 10,2 10,1 9,0 9,0 10,1 10)),((6.5 9.5,7 10,7 9,6 9,6 10,6.5 9.5)),((8 8,8 9,10 9,9 8,8 8)))",
    "MULTIPOLYGON(((0 7,0 8,1 8,1 7,0 7)),((5 3,4 3,4 4,6 4,6 3,7 3,6 2,5 2,5 3)),((8 2,8 3,9 2,8 2)),((1 1,2 2,2 1,1 1)),((2 1,3 1,2 0,1 0,2 1)),((2 3,3 4,3 3,2 3)),((1 9,2 8,1 8,1 9)),((2 10,2 9,1 9,1 10,2 10)),((9 7,9 8,10 8,10 7,9 7)),((6 0,6 1,7 1,7 0,6 0)),((8 0,9 1,9 0,8 0)),((1 6,1 5,0 5,1 6)),((0 2,1 1,0 1,0 2)),((1 3,2 3,2 2,1 2,1 3)),((5 1,5 0,4 0,4 1,3 1,4 2,5 2,6 1,5 1)),((1 3,0 3,0 4,1 4,1 3)),((3 6,4 5,2 5,3 6)),((9 2,10 2,10 1,9 1,9 2)),((7 5,6 4,6 5,7 6,8 6,8 5,7 5)),((7 4,8 5,8.5 4.5,9 5,9 4,8 4,8.5 3.5,9 4,10 3,7 3,7 4)),((1 6,1 7,3 7,3 8,4 7,5 7,6 8,6 10,7 9,8 10,9 10,9 9,8 9,8 8,7 8,6 7,6 6,1 6)))"
};


static std::string case_recursive_boxes_4[2] =
{
    // Occurred after refactoring assemble - not valid!
    "MULTIPOLYGON(((9 3,10 4,10 3,9 3)),((9 9,10 10,10 9,9 9)),((0 1,0 3,1 4,0 4,0 5,1 6,0 6,0 8,1 9,1 8,2 9,2 7,1.5 6.5,2.5 5.5,3 6,3 5,4 6,2 6,2 7,3 8,2 8,3 9,0 9,0 10,6 10,5.5 9.5,6 9,6 10,7 10,7.5 9.5,8 10,8 9,7 9,7 8,6 8,6.5 7.5,7 8,8 8,8 9,9 9,9.5 8.5,10 9,10 8,9.5 7.5,10 7,10 5,8 5,8 4,7 3,7 2,8 3,8 4,9 5,9 3,10 2,10 1,8 1,8.5 0.5,9 1,10 0,4 0,4 1,3 1,3 0,1 0,1 1,0 0,0 1),(4 2,4.5 1.5,5 2,5 4,4.5 3.5,5 3,4 3,4 2),(3 3,4 4,2 4,2.5 3.5,3 4,3 3),(6 4,6.5 3.5,7 4,6 4),(5 7,5 9,4 9,4 8,5 7)))",
    "MULTIPOLYGON(((1 0,2 1,2 0,1 0)),((7 9,7 10,8 10,7 9)),((1 0,0 0,0 3,1 3,2 2,2 3,1 3,1 4,2 4,2 5,1 4,0 4,0 8,1 7,1 6,2 7,1 7,1 9,0 9,0 10,7 10,6 9,6.5 8.5,7 9,8 9,9 8,8 8,9 7,9 6,10 7,10 5,9 5,9 4,10 5,10 0,7 0,8 1,7 1,6 0,3 0,3 1,1 1,1 0),(5 1,5.5 0.5,6 1,6 2,6.5 1.5,7 2,8 2,8 4,7 3,6 3,6 2,5 2,6 1,5 1),(4 4,5 4,5 5,4 4),(4 6,4 7,3 7,2 6,3 6,3 7,4 6),(6 5,6.5 4.5,7 5,6 5,7 6,7 7,6 7,6 5),(3.5 7.5,4 8,4 9,3 8,3.5 7.5)),((9 8,9 9,8 9,9 10,10 10,10 8,9 8)))"
};

static std::string case_recursive_boxes_5[2] =
{
    // Occurs after refactoring uu / handle_touch (not yet integrated)
    "MULTIPOLYGON(((0 9,0 10,1 10,1 9,0 9)),((9 0,9 1,10 1,10 0,9 0)),((5 6,5 7,6 7,6 6,7 6,7 4,6 4,6 5,5 5,5 6)),((5 3,7 3,7 2,4 2,4 3,5 3)),((5 8,5 9,7 9,7 8,5 8)),((4 0,1 0,1 1,5 1,5 0,4 0)),((3 5,3 4,4 4,4 3,2 3,2 2,1 2,1 3,0 3,0 4,2 4,2 5,1 5,1 6,4 6,4 5,3 5)),((0 2,1 2,1 1,0 1,0 2)),((4 10,4 7,1 7,1 6,0 6,0 8,1 8,1 9,2 9,2 10,4 10)),((9 4,9 3,8 3,8 5,9 5,9 4)),((7 2,8 2,8 0,7 0,7 2)),((8 7,10 7,10 6,7 6,7 8,8 8,8 7)))",
    "MULTIPOLYGON(((2 3,2 4,3 4,3 3,2 3)),((1 5,1 6,2 6,2 5,1 5)),((2 1,2 2,3 2,3 1,2 1)),((8 1,9 1,9 0,8 0,8 1)),((9 7,10 7,10 6,9 6,9 7)),((1 4,1 3,0 3,0 5,1 5,1 4)),((7 6,7 7,8 7,8 6,7 6)),((7 1,7 2,8 2,8 1,7 1)),((6 2,6 3,7 3,7 2,6 2)),((6 8,6 9,7 9,7 8,6 8)),((5 0,3 0,3 1,4 1,4 2,6 2,6 1,7 1,7 0,5 0)),((5 5,5 6,6 6,6 5,8 5,8 6,9 6,9 4,8 4,8 3,7 3,7 4,6 4,6 3,5 3,5 4,3 4,3 6,2 6,2 8,3 8,3 7,5 7,5 6,4 6,4 5,5 5)),((1 1,2 1,2 0,1 0,1 1)),((1 3,2 3,2 2,1 2,1 3)),((3 10,4 10,4 9,2 9,2 8,0 8,0 10,3 10)),((10 3,10 1,9 1,9 2,8 2,8 3,9 3,9 4,10 4,10 3)),((9 9,10 9,10 8,9 8,9 7,8 7,8 10,9 10,9 9)))"
};

static std::string case_recursive_boxes_6[2] =
{
    // Fixed by replacing handle_tangencies
    "MULTIPOLYGON(((2 2,1.5 1.5,2 1,1 1,2 0,0 0,0 3,1 3,1 4,2 4,2 5,5 5,5 0,2 0,2 2,3 1,3 2,3.5 1.5,4 2,2 2)),((1 5,2 5,0 3,0 5,1 5)))",
    "MULTIPOLYGON(((2 1,2 2,1 2,2 3,1 3,1 4,0 4,0 5,5 5,5 2,4 2,4 3,3 3,4 2,3 2,3 1,2 1)),((2 0,0 0,0 3,1 3,1 2,3 0,2 0)),((4 0,3 0,3 1,4 2,4 1,5 1,5 0,4 0)))"
};

static std::string case_recursive_boxes_7[2] =
{
    "MULTIPOLYGON(((3 1,3 2,4 2,4 1,3 1)),((2.5 2.5,3 3,3 2,2 2,2 3,2.5 2.5)),((2 1,3 0,1 0,1 2,2 2,2 1)))",
    "MULTIPOLYGON(((0 0,1 1,1 0,0 0)),((0 1,0 2,1 2,0 1)),((3.5 2.5,4 3,4 2,3 2,3 3,3.5 2.5)),((3 2,4 1,1 1,1 2,3 2)))"
};

static std::string case_recursive_boxes_8[2] =
{
    // Having colocated IP halfway segment
    "MULTIPOLYGON(((3 4,3 3,2 3,2 2,0 2,0 3,1 3,1 4,1.5 3.5,2 4,3 4)),((2 5,2 4,1 4,0 3,0 5,2 5)))",
    "MULTIPOLYGON(((3 4,4 4,4 3,3 3,3 1,0 1,0 2,1 2,1 3,0 2,0 4,3 4)))"
};

static std::string case_recursive_boxes_9[2] =
{
    // Needs ii turn skipping
    "MULTIPOLYGON(((2 2,3 2,3 0,2 0,2 1,1 1,2 2)),((1 1,1 0,0 0,0 3,0.5 2.5,1 3,2 2,1 2,1 1)))",
    "MULTIPOLYGON(((2 1,2 2,0 2,0 3,2 3,3 2,3 1,2 1)),((2.5 0.5,3 1,3 0,0 0,0 1,1 1,1 2,2.5 0.5)))"
};

static std::string case_recursive_boxes_10[4] =
{
    // Requires skipping ux for difference (a) and switching a->b
    "MULTIPOLYGON(((2 2,2 3,3 2,2 2)),((2 2,3 1,1 1,1 2,2 2)))",
    "MULTIPOLYGON(((3 2,2 1,2 3,3 3,3 2)))",

    // Inverse versions
    "MULTIPOLYGON(((0 0,0 4,4 4,4 0,0 0),(2 2,3 2,2 3,2 2),(2 2,1 2,1 1,3 1,2 2)))",
    "MULTIPOLYGON(((1 0,1 4,4 4,4 0,1 0),(3 2,3 3,2 3,2 1,3 2)))"
};

static std::string case_recursive_boxes_11[4] =
{
    // Requires switching a->b
    "MULTIPOLYGON(((3 2,5 2,5 1,4 1,4 0,3 0,3 1,2 1,3 2)))",
    "MULTIPOLYGON(((5 2,4 1,4 3,5 2)),((3 1,3 2,4 2,3 1)),((4 1,5 1,5 0,4 0,4 1)),((3 2,2 1,3 1,2 0,1 1,2 2,2 3,3 3,3 2)))",

    // Inverse versions
    "MULTIPOLYGON(((0 -1,0 4,6 4,6 -1,0 -1),(3 2,2 1,3 1,3 0,4 0,4 1,5 1,5 2,3 2)))",
    "MULTIPOLYGON(((0 -1,0 4,6 4,6 -1,0 -1),(5 2,4 3,4 1,5 2),(3 1,4 2,3 2,3 1),(4 1,4 0,5 0,5 1,4 1),(3 2,3 3,2 3,2 2,1 1,2 0,3 1,2 1,3 2)))"
};

static std::string case_recursive_boxes_12[2] =
{
    // One of them is invalid requiring discarding turns colocated with uu in these clusters
    "MULTIPOLYGON(((2 2,2 3,3 3,2 2)),((0 0,0 1,1 0,0 0)),((0 3,1 3,0.5 2.5,1 2,0 2,0 3)),((3 2,3 1,1 1,1 2,2 2,2 1,3 2)))",
    "MULTIPOLYGON(((0 1,0 2,1 2,0 1)),((0 1,1 1,1.5 0.5,2 1,2 0,0 0,0 1)),((1 3,1 4,2 3,1 2,1 3)))"
};

static std::string case_recursive_boxes_13[2] =
{
    // Strictly invalid, requires checking seg_id while considering skipping to next turn
    "MULTIPOLYGON(((2 1,2 2,3 2,3 1,2 1)),((3 2,3 3,4 3,4 2,3 2)),((2 4,1.5 3.5,2 3,1 3,2 2,0 2,1 3,1 5,2 5,2 4)))",
    "MULTIPOLYGON(((1 4,1 3,0 3,0 4,1 5,1 4)),((3 5,4 5,4 4,2 4,2 5,3 5)),((3 1,3 2,5 2,5 1,3 1)))"
};

static std::string case_recursive_boxes_14[2] =
{
    // Strictly invalid, requires skipping assignment of discarded turns for clusters
    "MULTIPOLYGON(((2 2,2 3,3 2,2 2)),((2 4,2 3,3 4,3 3,1 3,1 4,2 4)))",
    "MULTIPOLYGON(((3 3,4 4,4 3,3 3)),((1 2,2 3,2 2,1 2)),((2 1,2 2,3 1,2 1)),((1 4,1 5,2 5,2 4,1 4)))"
};

static std::string case_recursive_boxes_15[2] =
{
    // Requires inspecting blocked operations in traversing cluster
    "MULTIPOLYGON(((3 2,3 3,4 3,3 2)),((4 1,4 2,5 2,5 1,4 1)),((4 2,4 3,5 3,4 2)),((3 5,4 4,2 4,2 5,3 5)))",
    "MULTIPOLYGON(((3 4,4 3,4 4,5 4,5 3,4 3,3 2,3 3,2 3,2 4,3 4)))"
};

static std::string case_recursive_boxes_16[2] =
{
    // Requires inspecting if traverse is possible in selecting continue operation
    "MULTIPOLYGON(((2 4,1 3,0 3,0 5,3 5,3 4,4 4,4 5,5 5,5 4,4 3,2 3,3 4,2 4)),((2.5 1.5,3 1,3 2,4 2,4 1,5 2,5 0,3 0,3 1,2.5 0.5,3 0,0 0,0 2,3 2,2.5 1.5)))",
    "MULTIPOLYGON(((2 1,2 2,3 1,2 1)),((3 5,5 5,5 4,4 4,4 3,5 3,5 2,4 1,4 2,3 2,3 4,2 4,2 5,3 5)),((2 2,1 1,1 2,0 1,1 1,1 0,0 0,0 5,1 5,1 3,2 3,2 2)),((4 1,5 1,5 0,3 0,3 2,4 1)))"
};

static std::string case_recursive_boxes_17[2] =
{
    // Requires including uu turns, at least in clusters
    "MULTIPOLYGON(((0 4,0 5,1 5,0 4)),((4 0,5 1,5 0,4 0)),((4 5,5 5,5 4,3 4,3 5,4 5)),((3 4,3 3,2 2,1 2,1 1,2 1,2 2,2.5 1.5,4 3,5 3,5 1,4 1,4 0,3 0,3 1,2 1,3 0,0 0,0 2,1 2,1 5,1.5 4.5,2 5,2 4,3 4),(2 3,2 4,1 3,2 3)))",
    "MULTIPOLYGON(((2 5,3 5,2 4,2 5)),((3 1,4 2,4 0,3 0,3 1)),((2 0,0 0,0 1,1 2,0 2,1 3,2 2,2 3,3 2,3 1,2 0)),((1 4,0.5 3.5,1 3,0 3,0 4,1 4)),((4 3,3 3,3 5,4 5,4 4,5 4,5 2,4 2,4 3)))"
};

static std::string case_recursive_boxes_18[2] =
{
    // Simple case having two colocated uu turns
    "MULTIPOLYGON(((2 1,3 0,2 0,2 1)),((2 1,1 1,1 2,2 1)))",
    "MULTIPOLYGON(((2 2,2 3,3 3,3 2,2 1,2 2)))"
};

static std::string case_recursive_boxes_19[2] =
{
    // Simple case having two colocated uu and ux turns
    "MULTIPOLYGON(((1 4,2 3,1 3,1 4)),((1 4,0 4,0 5,1 4)))",
    "MULTIPOLYGON(((3 4,1 4,2 5,3 4)),((1 4,1 3,0 3,1 4)))"
};

static std::string case_recursive_boxes_20[2] =
{
    // Simple case having two colocated uu and (discarded) cc turns
    "MULTIPOLYGON(((4 4,4 5,5 5,5 4,4 4)),((4 4,4 3,3 3,3 4,4 4)))",
    "MULTIPOLYGON(((4 4,4 3,3 3,4 4)),((4 4,4 5,5 4,4 4)))"
};

static std::string case_recursive_boxes_21[2] =
{
    // Having colocated uu/ux/cc turns requiring traversing through arcs to
    // find first open outgoing arc for union
    "MULTIPOLYGON(((3 1,3 2,4 1,3 1)),((3 1,3 0,2 0,2 1,3 1)))",
    "MULTIPOLYGON(((3 1,3 0,2 0,3 1)),((3 1,2 1,2 2,3 2,3 1)))"
};

static std::string case_recursive_boxes_22[2] =
{
    // Requires including ux turns for intersections to block paths
    "MULTIPOLYGON(((2.5 1.5,3 1,2 1,2 3,3 3,2 2,3 2,2.5 1.5)))",
    "MULTIPOLYGON(((1 2,0 2,0 3,1 2)),((1 2,2 3,2 1,1 1,1 2)))"
};

static std::string case_recursive_boxes_23[2] =
{
    // [0] is invalid, [1] is valid
    // Requires discarding turns with uu for intersection/difference too
    "MULTIPOLYGON(((4.5 3.5,5 4,5 3,4 3,5 2,4 2,4 4,4.5 3.5)))",
    "MULTIPOLYGON(((4 3,5 4,5 3,4 3)),((3 3,3 4,4 3,3 3)))"
};

static std::string case_recursive_boxes_24[2] =
{
    // Requires including all combinations in clusters having uu
    "MULTIPOLYGON(((0 2,0 3,1 2,0 2)),((2 3,1 3,1 4,2 4,2 3)),((2 3,4 3,3 2,2 2,2 3)))",
    "MULTIPOLYGON(((3 2,4 1,2 1,3 2)),((3 2,2 2,2 3,3 2)),((2 2,2 1,1 1,1 2,2 2)))"
};

static std::string case_recursive_boxes_25[2] =
{
    // Requires startable flag per operation, assigned per cluster
    "MULTIPOLYGON(((4 1,4 2,5 2,5 1,4 0,4 1)),((3 2,3 3,4 3,4 2,3 2)),((3 2,3 1,2 1,3 2)))",
    "MULTIPOLYGON(((4 2,4 1,3 1,4 2)),((4 2,4 3,5 2,4 2)),((3 1,1 1,1 2,3 2,3 1)))"
};

static std::string case_recursive_boxes_26[2] =
{
    // Requires not including uu outside clusters (or travel through them)
    "MULTIPOLYGON(((2 4,3 4,3 3,0 3,1 4,1 5,2 5,2 4)),((1 3,1 2,0 2,1 3)))",
    "MULTIPOLYGON(((2 3,0 3,0 4,1 4,1 5,3 5,3 4,2 4,2 3)),((2 3,3 2,2 2,2 3)))"
};

static std::string case_recursive_boxes_27[2] =
{
    // Combination of lonely uu-turn (which is discarded) and a cluster containing it
    "MULTIPOLYGON(((2 2,3 1,3 0,2 0,2 2)),((2 2,1 2,1 3,2 2)))",
    "MULTIPOLYGON(((1 2,0 1,0 2,1 2)),((2 1,2 0,1 0,1 1,2 2,2 1)),((1 3,2 2,1 2,1 3)),((1 3,0 3,1 4,1 3)))"
};

static std::string case_recursive_boxes_28[2] =
{
    // Requires startable flag per operation, assigned per cluster (as #25 but in a different configuration)
    "MULTIPOLYGON(((5 1,5 0,4 0,4 2,5 3,5 1)),((4 2,3 2,3 3,4 3,4 2)))",
    "MULTIPOLYGON(((2 2,2 3,3 3,4 2,3 1,2 1,2 2)),((3 4,4 3,3 3,3 4)),((4 2,5 2,4 1,4 2)))"
};

static std::string case_recursive_boxes_29[2] =
{
    // Requires not discarding colocated cc turns
    "MULTIPOLYGON(((2 3,2 4,1 4,2 5,4 5,4 4,3 4,3 3,2 3)),((3 3,4 3,4 1,5 1,5 0,3 0,3 2,2 2,3 3)),((1 2,0 2,0 3,1 3,1 2)),((1 1,0 1,1 2,2 2,2 1,1 1)),((2 1,2 0,1 0,2 1)))",
    "MULTIPOLYGON(((0 4,0 5,1 4,0 4)),((2 3,2 4,4 4,4 3,2 3)),((2 2,1 2,1 3,2 3,2 2)),((1 2,0 2,0 3,1 3,0.5 2.5,1 2)),((1 0,0 0,1 1,4 1,4 0,1 0)),((4 0,5 1,5 0,4 0)))"
};

static std::string case_recursive_boxes_30[2] =
{
    // Requires not discarding turns colocated with uu/invalid polygons (now not necessary anymore because of startable)
    "MULTIPOLYGON(((2 2,2 3,4 3,4 4,4.5 3.5,5 4,5 0,3 0,3 1,4 1,4 2,2 2)),((1 5,3 5,4 4,0 4,0 5,1 5)))",
    "MULTIPOLYGON(((2 1,2 3,1 3,1 4,2 5,2 4,3 4,3 5,5 5,5 4,4 4,3.5 3.5,4 3,4 4,5 3,4.5 2.5,5 2,5 0,4 0,4 2,3 2,3 3,2.5 2.5,4 1,3 1,3 0,1 0,2 1)))"
};

static std::string case_recursive_boxes_31[2] =
{
    // Requires allowing traverse through clusters having only uu/cc for intersection
    "MULTIPOLYGON(((1 4,1 1,0 1,0 4,1 4)),((1 1,2 1,2 0,1 0,1 1)),((2 2,1 2,2 3,2 2)))",
    "MULTIPOLYGON(((2 3,2 2,1 2,2 3)),((0 1,0 3,1 3,1 1,0 1)),((1 1,1 0,0 0,1 1)))"
};

static std::string case_recursive_boxes_32[2] =
{
    // Similar to #31 but here uu/ux/cc
    "MULTIPOLYGON(((1 3,2 3,2 2,1 1,1 3)),((2 2,3 1,3 0,2 0,2 2)),((1 1,2 1,1 0,0 0,0 1,1 1)))",
    "MULTIPOLYGON(((3 1,3 0,2 0,2 2,3 1)),((1 1,0 1,0 2,1 2,2 1,1 1)))"
};

static std::string case_recursive_boxes_33[2] =
{
    // Similar to #31 but here also a ui
    "MULTIPOLYGON(((4 3,5 3,5 2,3 2,4 3)),((4 2,5 1,3 1,4 2)),((2 1,3 1,3 0,2 0,2 1)),((3 2,1 2,1 3,2 3,3 2)))",
    "MULTIPOLYGON(((3 1,2 1,2 4,4 4,4 3,5 2,3 2,3 1)),((3 1,4 2,4 1,5 1,5 0,3 0,3 1)))"
};

static std::string case_recursive_boxes_34[2] =
{
    // Requires detecting finished arcs during cluster traversal
    "MULTIPOLYGON(((2 0,0 0,0 5,2 5,2 4,3 5,5 5,5 0,2 0)))",
    "MULTIPOLYGON(((3 3,2 3,2 2,3 2,3 1,2 0,1 0,2 1,1 1,1 0,0 0,0 5,1 4,1 5,4 5,3 4,3.5 3.5,4 4,4 5,5 4,5 1,4 1,4 3,3 3)),((3 1,4 1,3 0,3 1)))"
};

static std::string case_recursive_boxes_35[2] =
{
    // Requires detecting finished arcs during cluster traversal
    "MULTIPOLYGON(((3 1,2 1,3 0,0 0,0 2,1 2,1 3,0 2,0 5,4 5,4 4,5 4,5 1,4 1,5 0,3 0,3 1),(2.5 1.5,3 2,2 2,2.5 1.5),(2 4,1 4,2 3,2 4)))",
    "MULTIPOLYGON(((2 2,1.5 1.5,2 1,1 1,1 0,0 0,0 5,3 5,2.5 4.5,3 4,3 5,5 5,5 0,1 0,2 1,2 2),(3 2,3 3,2.5 2.5,3 2),(2 3,2 4,1 4,1 3,2 3)))"
};

static std::string case_recursive_boxes_36[2] =
{
    // Requires increasing target rank while skipping finished arcs to avoid duplicate output
    "MULTIPOLYGON(((5 3,4 3,4 4,5 3)),((5 2,4 2,5 3,5 2)),((5 2,5 1,4 1,5 2)))",
    "MULTIPOLYGON(((4 2,4 3,5 3,5 2,4 2)),((4 2,4 1,3 1,3 2,4 2)))"
};

static std::string pie_21_7_21_0_3[2] =
{
    "MULTIPOLYGON(((2500 2500,2500 3875,2855 3828,3187 3690,3472 3472,3690 3187,3828 2855,3875 2500,3828 2144,3690 1812,3472 1527,3187 1309,2855 1171,2499 1125,2144 1171,1812 1309,1527 1527,1309 1812,1171 2144,1125 2499,1171 2855,1309 3187,2500 2500)))",
    "MULTIPOLYGON(((2500 2500,1704 3295,1937 3474,2208 3586,2499 3625,2791 3586,3062 3474,3295 3295,2500 2500)),((2500 2500,3586 2791,3625 2500,3586 2208,2500 2500)))"
};

static std::string pie_23_19_5_0_2[2] =
{
    "MULTIPOLYGON(((2500 2500,2500 3875,2855 3828,3187 3690,3472 3472,3690 3187,3828 2855,3875 2500,3828 2144,3690 1812,3472 1527,3187 1309,2855 1171,2499 1125,2144 1171,1812 1309,1527 1527,1309 1812,1171 2144,1125 2499,1171 2855,1309 3187,1527 3472,1812 3690,2500 2500)))",
    "MULTIPOLYGON(((2500 2500,3586 2791,3625 2500,3586 2208,3474 1937,3295 1704,3062 1525,2791 1413,2499 1375,2208 1413,1937 1525,1704 1704,1525 1937,1413 2208,1375 2500,1413 2791,1525 3062,1704 3295,1937 3474,2208 3586,2500 2500)),((2500 2500,2791 3586,3062 3474,2500 2500)))"
};

static std::string pie_7_14_5_0_7[2] =
{
    "MULTIPOLYGON(((2500 2500,2500 3875,2855 3828,3187 3690,3472 3472,3690 3187,3828 2855,3875 2500,2500 2500)))",
    "MULTIPOLYGON(((2500 2500,3586 2791,3625 2500,3586 2208,3474 1937,3295 1704,3062 1525,2791 1413,2499 1375,2208 1413,1937 1525,1704 1704,1525 1937,1413 2208,1375 2500,2500 2500)),((2500 2500,1525 3062,1704 3295,1937 3474,2208 3586,2499 3625,2791 3586,3062 3474,2500 2500)))"
};

static std::string pie_16_16_9_0_2[2] =
{
    "MULTIPOLYGON(((2500 2500,2500 3875,2855 3828,3187 3690,3472 3472,3690 3187,3828 2855,3875 2500,3828 2144,3690 1812,3472 1527,3187 1309,2855 1171,2499 1125,2144 1171,1812 1309,1527 1527,2500 2500)))",
    "MULTIPOLYGON(((2500 2500,3295 1704,3062 1525,2791 1413,2499 1375,2208 1413,1937 1525,1704 1704,1525 1937,1413 2208,1375 2500,1413 2791,1525 3062,1704 3295,1937 3474,2208 3586,2499 3625,2500 2500)),((2500 2500,3062 3474,3295 3295,2500 2500)))"
};

static std::string pie_7_2_1_0_15[2] =
{
    "MULTIPOLYGON(((2500 2500,2500 3875,2855 3828,3187 3690,3472 3472,3690 3187,3828 2855,3875 2500,2500 2500)))",
    "MULTIPOLYGON(((2500 2500,2791 3586,3062 3474,2500 2500)),((2500 2500,3474 3062,3586 2791,3625 2500,3586 2208,3474 1937,3295 1704,3062 1525,2791 1413,2499 1375,2208 1413,1937 1525,1704 1704,1525 1937,1413 2208,1375 2500,2500 2500)))"
};

// Case, not literally on this list but derived, to mix polygon/multipolygon in call to difference
static std::string ggl_list_20111025_vd[4] =
    {
    "POLYGON((0 0,0 4,4 0,0 0))",
    "POLYGON((10 0,10 5,15 0,10 0))",
    "MULTIPOLYGON(((0 0,0 4,4 0,0 0)))",
    "MULTIPOLYGON(((10 0,10 5,15 0,10 0)))"
    };

// Same, mail with other case with text "Say the MP is the 2 squares below and P is the blue-ish rectangle."
static std::string ggl_list_20111025_vd_2[2] =
    {
    "POLYGON((5 0,5 4,8 4,8 0,5 0))",
    "MULTIPOLYGON(((0 0,0 2,2 2,2 0,0 0)),((4 0,4 2,6 2,6 0,4 0)))"
    };

// Mail of h2 indicating that reversed order (in second polygon) has ix/ix problems
static std::string ggl_list_20120915_h2[3] =
    {
        "MULTIPOLYGON(((-2 5, -1 5, 0 5, 2 5, 2 -2, 1 -2, 1 -1, 0 -1,0 0, -1 0, -2 0, -2 5)))",
        "MULTIPOLYGON(((0 0, 1 0, 1 -1, 0 -1, 0 0)), ((-1 5, 0 5, 0 0, -1 0, -1 5)))",
        "MULTIPOLYGON(((-1 5, 0 5, 0 0, -1 0, -1 5)), ((0 0, 1 0, 1 -1, 0 -1, 0 0)))"
    };

// Mail of volker, about another problem, but this specific example is causing two-point inner rings polygons which should be discarded
// (condition of num_points in detail/overlay/convert_ring.hpp)
static std::string ggl_list_20120221_volker[2] =
    {
        "MULTIPOLYGON(((1032 2130,1032 1764,2052 2712,1032 2130)),((3234 2580,2558 2690,3234 2532,3234 2580)),((2558 2690,2136 2790,2052 2712,2136 2760,2558 2690)))",
        "MULTIPOLYGON(((3232 2532.469945355191,2136 2790,1032 1764,1032 1458,1032 1212,2136 2328,3232 2220.196721311475,3232 1056,1031 1056,1031 2856,3232 2856,3232 2532.469945355191),(3232 2412.426229508197,2136 2646,3232 2412.426229508197)))"
    };

static std::string ggl_list_20140212_sybren[2] =
    {
        "MULTIPOLYGON(((0.494062 0.659354,0.471383 0.64654,0.446639 0.616561,0.47291 0.61171,0.495396 0.625263,0.494964 0.679709,0.494062 0.659354)))",
        "MULTIPOLYGON(((0.4951091661995328 0.6614133543986973,0.495396 0.625263,0.50092 0.6492750000000001,0.494964 0.679709,0.477258 0.698703,0.4951091661995328 0.6614133543986973)),((0.452167 0.706562,0.433379 0.696888,0.442673 0.65792,0.464729 0.671387,0.452167 0.706562)))"
    };

static std::string ticket_9081[2] =
    {
        "MULTIPOLYGON(((0.5489109414010371 0.5774835110050927,0.4099611282054447 0.4644351568071598,0.4294011278595494 0.4843224236729239,0.4205359995313906 0.5115225580860201,0.4441572412013468 0.5184999851878852,0.5489109414010371 0.5774835110050927)),((0.562085028126843 0.5882018328808966,0.5644349663154944 0.591180348361206,0.568218114394707 0.5970364466647042,0.5838690879677763 0.6212632646137447,0.5873787029417971 0.6412877041753083,0.468699602592386 0.5866280231830688,0.4171010902425981 0.5220616039851281,0.4059124592966251 0.5563907478354578,0.3909547828925878 0.6022841397455458,0.520859401226844 0.9508041627246925,0.8595233008819849 0.8301950132755517,0.562085028126843 0.5882018328808966)))",
        "MULTIPOLYGON(((0.2099392122251989 0.492066865490789,0.1124301889095737 0.5124668111209448,0.3306914939102383 0.6126684490171914,0.2099392122251989 0.492066865490789)),((0.5885369465145437 0.6478961722242873,0.5342320718598281 0.6686303269145104,0.5619623880692838 0.7033299168703926,0.5945761233023867 0.6823532655194001,0.5885369465145437 0.6478961722242873)),((0.5570738195183501 0.6001870087680015,0.5429714753344335 0.6231021858940831,0.5880357506342242 0.6450365518134291,0.5838690879677763 0.6212632646137447,0.568218114394707 0.5970364466647042,0.5570738195183501 0.6001870087680015)),((0.5498478321815098 0.5029279381860542,0.608691671498764 0.5163121433149205,0.5636607291345047 0.5894838094559455,0.8595233008819849 0.8301950132755517,0.8285440738598029 0.8412277162756114,0.9591357158116398 0.9011810663167211,0.8572649311807611 0.3566393017365032,0.5965816668471951 0.4111770689940296,0.5498478321815098 0.5029279381860542)),((0.3984249865018206 0.4526335964808558,0.3621206996557855 0.4602288471829723,0.4183516736935784 0.4730187483833363,0.4099611282054451 0.4644351568071601,0.3984249865018206 0.4526335964808558)))"
    };

// Integer, ccw, open
static std::string ticket_10661[3] =
    {
        /* A */ "MULTIPOLYGON(((1701 985,3501 985,3501 2785,1701 2785,1701 985)))",
        /* B */ "MULTIPOLYGON(((1698 1860,1698 1122,2598 1392,3492 1842,3492 32706,2598 2340,1698 1860)))",
        /* C=A-B, */
        /* D */ "MULTIPOLYGON(((1698 2772,1698 1860,2598 2340,3492 2412,3492 32743,1698 2772)))"
        // Reported problem was: validity of difference C-D
    };

// Integer, ccw, open
static std::string ticket_10803[2] =
    {
        "MULTIPOLYGON(((3174 1374,3174 2886,1374 2886,1374 2139,3174 1374)))",
        "MULTIPOLYGON(((1374 1092,1734 1092,3174 2526,3174 2886,1374 2886,1374 1092)))"
    };

// Integer, ccw, open
static std::string ticket_11674[2] =
    {
        "MULTIPOLYGON(((529 3217,529 998,5337 998,5337 1834,5070 2000,5337 2072,5337 3475,529 3475,529 3312,1734 2054,2934 1670,3230 1690,2934 1400,1734 1784,529 3217),(4140 2582,5071 2001,4140 1754,3231 1691,4140 2582)))",
        "MULTIPOLYGON(((528 3218,528 2498,1734 1406,2556 1522,1734 1784,528 3218)),((4610 2288,5340 1178,5340 1832,4609 2289,4140 3002,2934 1574,2555 1521,2934 1400,4140 2582,4610 2288)))",
    };

static std::string bug_21155501[2] =
    {
        "MULTIPOLYGON(((-8.3935546875 27.449790329784214,4.9658203125 18.729501999072138,11.8212890625 23.563987128451217,9.7119140625 25.48295117535531,9.8876953125 31.728167146023935,8.3056640625 32.99023555965106,8.5693359375 37.16031654673677,-1.8896484375 35.60371874069731,-0.5712890625 32.02670629333614,-8.9208984375 29.458731185355344,-8.3935546875 27.449790329784214)))",
        "MULTIPOLYGON(((4.9658203125 18.729501999072138,-3.4868710311820115 24.246968623627644,8.3589904332912 33.833614418115445,8.3056640625 32.99023555965106,9.8876953125 31.728167146023935,9.7119140625 25.48295117535531,11.8212890625 23.563987128451217,4.9658203125 18.729501999072138)),((-3.88714525609152 24.508246314579743,-8.3935546875 27.449790329784214,-8.9208984375 29.458731185355344,-0.5712890625 32.02670629333614,-1.8896484375 35.60371874069731,8.5693359375 37.16031654673677,8.362166569827938 33.883846345901595,-3.88714525609152 24.508246314579743)))",
    };

static std::string mysql_21965285_b[2] =
    {
        "MULTIPOLYGON(((3 0, -19 -19, -7 3, -2 10, 15 0, 3 0)))",
        "MULTIPOLYGON(((1 1, 3 0, 19 -8, -4 -3, 1 1)),((3 0, -2 7, -3 16, 1 19, 8 12, 3 0)))"
    };

#endif // BOOST_GEOMETRY_TEST_MULTI_OVERLAY_CASES_HPP
