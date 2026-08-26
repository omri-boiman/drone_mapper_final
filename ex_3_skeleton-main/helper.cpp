#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <memory>

enum class Direction { North, East, South, West };
enum class Step { North, East, South, West, Stay, Finish };

class WallsSensor {
public:
	virtual ~WallsSensor() {}
	virtual bool isWall(Direction d) const = 0;
};

class DirtSensor {
public:
	virtual ~DirtSensor() {}
	virtual int dirtLevel() const = 0;
};

class BatteryMeter {
public:
	virtual ~BatteryMeter() {}
	virtual std::size_t getBatteryState() const = 0;
};

class AbstractAlgorithm {
public:
	virtual ~AbstractAlgorithm() {}
	virtual void setMaxSteps(std::size_t maxSteps) = 0;
	virtual void setWallsSensor(const WallsSensor&) = 0;
	virtual void setDirtSensor(const DirtSensor&) = 0;
	virtual void setBatteryMeter(const BatteryMeter&) = 0;
	virtual Step nextStep() = 0;
};

//-------------------------------
// common/AlgorithmRegistrar.h
//-------------------------------
using AlgorithmFactory = std::function<std::unique_ptr<AbstractAlgorithm>()>;

class AlgorithmRegistrar {
    class AlgorithmFactoryPair {
        std::string name_;
        AlgorithmFactory algorithmFactory_;
    public:
        AlgorithmFactoryPair(const std::string& name, AlgorithmFactory algorithmFactory)
            : name_(name), algorithmFactory_(std::move(algorithmFactory)) {}
        // NOTE: API is guaranteed, actual implementation may change
        const std::string& name() const { return name_; }
        std::unique_ptr<AbstractAlgorithm> create() const { return algorithmFactory_(); }
    };
    std::vector<AlgorithmFactoryPair> algorithms;
    static AlgorithmRegistrar registrar;
public:
    // NOTE: API is guaranteed, actual implementation may change
    static AlgorithmRegistrar& getAlgorithmRegistrar();
    void registerAlgorithm(const std::string& name, AlgorithmFactory algorithmFactory) {
        algorithms.emplace_back(name, std::move(algorithmFactory));
    }
    auto begin() const {
        return algorithms.begin();
    }
    auto end() const {
        return algorithms.end();
    }
    std::size_t count() const { return algorithms.size(); }
    void clear() { algorithms.clear(); }
};

//-----------------------------------
// simulator/AlgorithmRegistrar.cpp
//-----------------------------------
AlgorithmRegistrar AlgorithmRegistrar::registrar;

AlgorithmRegistrar& AlgorithmRegistrar::getAlgorithmRegistrar() {
    return registrar;
}

//------------------------------------
// algorithm/AlgorithmRegistration.h
//------------------------------------
struct AlgorithmRegistration {
    AlgorithmRegistration(const std::string& name, AlgorithmFactory algorithmFactory) {
        AlgorithmRegistrar::getAlgorithmRegistrar().registerAlgorithm(name, std::move(algorithmFactory));
    }
};

#define REGISTER_ALGORITHM(ALGO) AlgorithmRegistration ALGO##_obj(#ALGO, []{return std::make_unique<ALGO>();})

//-------------------------------
// algorithm/Algo_123456789.h
//-------------------------------
class Algo_123456789: public AbstractAlgorithm {
public:
	void setMaxSteps(std::size_t maxSteps) override {}
	void setWallsSensor(const WallsSensor&) override {}
	void setDirtSensor(const DirtSensor&) override {}
	void setBatteryMeter(const BatteryMeter&) override {}
	Step nextStep() override {
        return Step::North;	    
	}
};

//-------------------------------
// algorithm/Algo_123456789.cpp
//-------------------------------
REGISTER_ALGORITHM(Algo_123456789);

//-------------------------------
// algorithm/Algo_987654321.h
//-------------------------------
class Algo_987654321: public AbstractAlgorithm {
public:
	void setMaxSteps(std::size_t maxSteps) override {}
	void setWallsSensor(const WallsSensor&) override {}
	void setDirtSensor(const DirtSensor&) override {}
	void setBatteryMeter(const BatteryMeter&) override {}
	Step nextStep() override {
        return Step::South;	    
	}
};

//-------------------------------
// algorithm/Algo_987654321.cpp
//-------------------------------
REGISTER_ALGORITHM(Algo_987654321);

//-------------------------------
// main.cpp
//-------------------------------
int main() {
    // dlopen
    for(const auto& algo: AlgorithmRegistrar::getAlgorithmRegistrar()) {
        auto algorithm = algo.create();
        std::cout << algo.name() << ": " << static_cast<int>(algorithm->nextStep()) << std::endl;
    }
    AlgorithmRegistrar::getAlgorithmRegistrar().clear();
    // dlclose
}
