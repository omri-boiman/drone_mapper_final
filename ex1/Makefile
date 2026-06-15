BUILD_DIR  := build
CXX        := g++
CXXFLAGS   := -std=c++20 -Wall -Wextra -Werror -pedantic

.PHONY: all clean rebuild

all:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release \
	      -DCMAKE_CXX_COMPILER=$(CXX) \
	      -DCMAKE_CXX_FLAGS="$(CXXFLAGS)"
	cmake --build $(BUILD_DIR)

clean:
	cmake --build $(BUILD_DIR) --target clean

rebuild:
	rm -rf $(BUILD_DIR)
	$(MAKE) all
