#include <gtest/gtest.h>

#include <UserCommon/ErrorLogger.h>
#include <UserCommon/GridCell3D.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>

// UserCommon (GridCell3D, ErrorLogger) had zero dedicated tests anywhere in
// the ex3 tree -- both were only exercised indirectly through consumers
// (MapsComparison/MappingAlgorithm for GridCell3D; nothing for ErrorLogger at
// all). See ex3-test-plan.md, section 7.

namespace uc = user_common_211781141_325049575;

// ---- GridCell3D ----

TEST(GridCell3D, EqualityComparesAllThreeFields) {
    EXPECT_EQ((uc::GridCell3D{1, 2, 3}), (uc::GridCell3D{1, 2, 3}));
    EXPECT_NE((uc::GridCell3D{1, 2, 3}), (uc::GridCell3D{9, 2, 3}));
    EXPECT_NE((uc::GridCell3D{1, 2, 3}), (uc::GridCell3D{1, 9, 3}));
    EXPECT_NE((uc::GridCell3D{1, 2, 3}), (uc::GridCell3D{1, 2, 9}));
}

TEST(GridCell3D, DefaultConstructedIsOrigin) {
    EXPECT_EQ(uc::GridCell3D{}, (uc::GridCell3D{0, 0, 0}));
}

TEST(GridCell3D, LessThanIsLexicographicByXThenYThenZ) {
    EXPECT_LT((uc::GridCell3D{0, 9, 9}), (uc::GridCell3D{1, 0, 0}));
    EXPECT_LT((uc::GridCell3D{1, 0, 9}), (uc::GridCell3D{1, 1, 0}));
    EXPECT_LT((uc::GridCell3D{1, 1, 0}), (uc::GridCell3D{1, 1, 1}));
    EXPECT_FALSE((uc::GridCell3D{1, 1, 1}) < (uc::GridCell3D{1, 1, 1}));
}

TEST(GridCell3D, HashIsConsistentAndUsableInUnorderedContainers) {
    const uc::GridCell3D a{4, -2, 7};
    const uc::GridCell3D a_copy{4, -2, 7};
    const uc::GridCell3D b{4, -2, 8};

    // Equal objects must hash equal (required for correct unordered_set use).
    EXPECT_EQ(std::hash<uc::GridCell3D>{}(a), std::hash<uc::GridCell3D>{}(a_copy));

    std::unordered_set<uc::GridCell3D> set;
    set.insert(a);
    set.insert(a_copy); // duplicate, should not grow the set
    set.insert(b);
    EXPECT_EQ(set.size(), 2u);
    EXPECT_TRUE(set.count(a));
    EXPECT_TRUE(set.count(b));
    EXPECT_FALSE(set.count(uc::GridCell3D{0, 0, 0}));
}

// ---- ErrorLogger ----

class ErrorLogger : public ::testing::Test {
protected:
    void SetUp() override {
        tmp_dir_ = std::filesystem::temp_directory_path() / "ex3_error_logger_test";
        std::filesystem::remove_all(tmp_dir_);
        std::filesystem::create_directories(tmp_dir_);
    }
    void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

    std::filesystem::path tmp_dir_;
};

TEST_F(ErrorLogger, NoLogCalls_HasErrorsFalse) {
    uc::ErrorLogger logger(tmp_dir_ / "error.log");
    EXPECT_FALSE(logger.hasErrors());
}

TEST_F(ErrorLogger, AfterOneLogCall_HasErrorsTrue) {
    uc::ErrorLogger logger(tmp_dir_ / "error.log");
    logger.log("something went wrong");
    EXPECT_TRUE(logger.hasErrors());
}

TEST_F(ErrorLogger, LoggedMessageAppearsInFile) {
    const auto path = tmp_dir_ / "error.log";
    {
        uc::ErrorLogger logger(path);
        logger.log("DRONE_HITS_OBSTACLE");
    }
    ASSERT_TRUE(std::filesystem::exists(path));

    std::ifstream in(path);
    std::ostringstream contents;
    contents << in.rdbuf();
    EXPECT_NE(contents.str().find("DRONE_HITS_OBSTACLE"), std::string::npos) << contents.str();
}

TEST_F(ErrorLogger, MultipleMessagesAllAppended) {
    const auto path = tmp_dir_ / "error.log";
    {
        uc::ErrorLogger logger(path);
        logger.log("first error");
        logger.log("second error");
    }
    std::ifstream in(path);
    std::ostringstream contents;
    contents << in.rdbuf();
    const std::string text = contents.str();
    EXPECT_NE(text.find("first error"), std::string::npos) << text;
    EXPECT_NE(text.find("second error"), std::string::npos) << text;
    // Two distinct lines, not one overwriting the other.
    EXPECT_EQ(std::count(text.begin(), text.end(), '\n'), 2);
}

TEST_F(ErrorLogger, CreatesParentDirectoriesIfMissing) {
    const auto nested = tmp_dir_ / "nested" / "dirs" / "error.log";
    ASSERT_FALSE(std::filesystem::exists(nested.parent_path()));
    uc::ErrorLogger logger(nested);
    logger.log("x");
    EXPECT_TRUE(std::filesystem::exists(nested));
}
