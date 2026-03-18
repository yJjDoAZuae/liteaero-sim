#include "DynamicElement.hpp"

#include <stdexcept>
#include <string>

namespace liteaerosim {

void DynamicElement::initialize(const nlohmann::json& config) {
    validateSchema(config);
    onInitialize(config);
}

void DynamicElement::reset() {
    onReset();
}

nlohmann::json DynamicElement::serializeJson() const {
    nlohmann::json state = onSerializeJson();
    state["schema_version"] = schemaVersion();
    state["type"]           = typeName();
    return state;
}

void DynamicElement::deserializeJson(const nlohmann::json& state) {
    validateSchema(state);
    onDeserializeJson(state);
}

void DynamicElement::attachLogger(ILogger* logger) noexcept {
    logger_ = logger;
}

void DynamicElement::validateSchema(const nlohmann::json& state) const {
    if (!state.contains("schema_version")) {
        return;  // Config objects may not carry schema_version; only state snapshots do
    }
    const int stored  = state.at("schema_version").get<int>();
    const int current = schemaVersion();
    if (stored != current) {
        throw std::runtime_error(
            std::string("DynamicElement::deserializeJson: schema version mismatch for ") +
            typeName() + ": stored=" + std::to_string(stored) +
            " current=" + std::to_string(current));
    }
}

} // namespace liteaerosim
