#include <gtest/gtest.h>

#include "RunModes.h"

// Unit tests for simulator::partitionWork (Simulator/src/RunModes.cpp,
// extracted out of main.cpp's anonymous namespace specifically so it could be
// tested directly -- see ex3-test-plan.md, section 5, and the "refactor
// note" it calls out). Previously zero coverage of any kind -- the threading
// behavior was only checked empirically via wall-clock timing on a couple of
// manual runs (see memory-ex3-status.md).

namespace sim = simulator;

namespace {

// Every chunk must be contiguous, disjoint, cover exactly [0, total_items),
// and no chunk may be empty (an empty chunk would mean a spawned thread with
// nothing to do, which the spec explicitly forbids: "you should not open
// threads which cannot be utilized").
void expectValidPartition(const std::vector<std::pair<std::size_t, std::size_t>>& chunks,
                          std::size_t total_items) {
    std::size_t expected_next = 0;
    for (const auto& [begin, end] : chunks) {
        EXPECT_EQ(begin, expected_next);
        EXPECT_LT(begin, end) << "chunk must not be empty";
        expected_next = end;
    }
    EXPECT_EQ(expected_next, total_items);
}

} // namespace

class PartitionWork : public ::testing::Test {};

TEST_F(PartitionWork, ZeroItems_EmptyResult) {
    const auto chunks = sim::partitionWork(0, std::nullopt);
    EXPECT_TRUE(chunks.empty());
}

TEST_F(PartitionWork, ZeroItems_EmptyResult_EvenWithThreadsRequested) {
    const auto chunks = sim::partitionWork(0, 8u);
    EXPECT_TRUE(chunks.empty());
}

TEST_F(PartitionWork, OneItem_SingleChunk_RegardlessOfThreadsRequested) {
    for (auto requested : {std::optional<unsigned>{}, std::optional<unsigned>{1},
                            std::optional<unsigned>{2}, std::optional<unsigned>{100}}) {
        const auto chunks = sim::partitionWork(1, requested);
        ASSERT_EQ(chunks.size(), 1u);
        EXPECT_EQ(chunks[0], std::make_pair(std::size_t{0}, std::size_t{1}));
    }
}

TEST_F(PartitionWork, NumThreadsAbsentOrOne_AlwaysSingleThreaded) {
    for (auto requested : {std::optional<unsigned>{}, std::optional<unsigned>{1}}) {
        for (std::size_t items : {2u, 5u, 100u}) {
            const auto chunks = sim::partitionWork(items, requested);
            ASSERT_EQ(chunks.size(), 1u) << "items=" << items;
            expectValidPartition(chunks, items);
        }
    }
}

TEST_F(PartitionWork, TwoItems_NeverProducesExactlyTwoThreads) {
    // The spec's core invariant: "the total number of threads will never be
    // 2." With exactly 2 work items, any num_threads>=2 request would
    // naively want 1 extra thread (+1 main = 2 total) -- this must collapse
    // to single-threaded instead.
    for (unsigned requested : {2u, 3u, 5u, 100u}) {
        const auto chunks = sim::partitionWork(2, requested);
        ASSERT_EQ(chunks.size(), 1u) << "requested=" << requested;
        expectValidPartition(chunks, 2);
    }
}

TEST_F(PartitionWork, ThreeItems_TwoExtraThreadsRequested_AllThreeUtilized) {
    // 3 items, 2 extra threads requested -> extra_threads = min(2, 2) = 2,
    // not the collapse case (only ==1 collapses) -> 3 total workers, one
    // item each.
    const auto chunks = sim::partitionWork(3, 2u);
    ASSERT_EQ(chunks.size(), 3u);
    EXPECT_EQ(chunks[0], std::make_pair(std::size_t{0}, std::size_t{1}));
    EXPECT_EQ(chunks[1], std::make_pair(std::size_t{1}, std::size_t{2}));
    EXPECT_EQ(chunks[2], std::make_pair(std::size_t{2}, std::size_t{3}));
}

TEST_F(PartitionWork, FiveItems_TwoExtraThreads_RemainderDistributedToEarlyChunks) {
    // extra_threads = min(2, 4) = 2 -> 3 total workers. base = 5/3 = 1,
    // remainder = 2, so the first 2 chunks get one extra item each.
    const auto chunks = sim::partitionWork(5, 2u);
    ASSERT_EQ(chunks.size(), 3u);
    EXPECT_EQ(chunks[0], std::make_pair(std::size_t{0}, std::size_t{2}));
    EXPECT_EQ(chunks[1], std::make_pair(std::size_t{2}, std::size_t{4}));
    EXPECT_EQ(chunks[2], std::make_pair(std::size_t{4}, std::size_t{5}));
    expectValidPartition(chunks, 5);
}

TEST_F(PartitionWork, TenItems_ThreeExtraThreads_ExactBreakdown) {
    // extra_threads = min(3, 9) = 3 -> 4 total workers. base = 10/4 = 2,
    // remainder = 2 -> first two chunks get 3 items, last two get 2.
    const auto chunks = sim::partitionWork(10, 3u);
    ASSERT_EQ(chunks.size(), 4u);
    EXPECT_EQ(chunks[0], std::make_pair(std::size_t{0}, std::size_t{3}));
    EXPECT_EQ(chunks[1], std::make_pair(std::size_t{3}, std::size_t{6}));
    EXPECT_EQ(chunks[2], std::make_pair(std::size_t{6}, std::size_t{8}));
    EXPECT_EQ(chunks[3], std::make_pair(std::size_t{8}, std::size_t{10}));
    expectValidPartition(chunks, 10);
}

TEST_F(PartitionWork, RequestingMoreThreadsThanItemsMinusOne_CapsAtItemsMinusOne) {
    // "the exact number of threads may be lower than requested... you should
    // not open threads which cannot be utilized" -- 100 items, 1000 threads
    // requested -> capped at 99 extra (one work item per worker, 100 total).
    const auto chunks = sim::partitionWork(100, 1000u);
    ASSERT_EQ(chunks.size(), 100u);
    for (std::size_t i = 0; i < chunks.size(); ++i) {
        EXPECT_EQ(chunks[i], std::make_pair(i, i + 1));
    }
}

TEST_F(PartitionWork, NeverProducesExactlyTwoTotalWorkers_AcrossManyInputs) {
    // General invariant sweep, not just the item-count==2 special case above:
    // for no (items, requested) combination should the result ever have
    // exactly 2 chunks (which would mean exactly 2 threads total).
    for (std::size_t items = 0; items <= 20; ++items) {
        for (unsigned requested = 0; requested <= 6; ++requested) {
            const auto chunks = sim::partitionWork(items, requested);
            EXPECT_NE(chunks.size(), 2u)
                << "items=" << items << " requested=" << requested;
            if (!chunks.empty()) expectValidPartition(chunks, items);
        }
    }
}
