# spangap/audio — generic I2S audio engine + ITS packet server.
#
# Cross-straddle deps flow through ${SPANGAP_REQUIRES} only (the staging lint
# rejects a hand-written sibling repo name). esp_driver_i2s is the IDF I2S
# driver (i2s_std + i2s_pdm); the engine owns it so audio boards contribute
# only Kconfig data + an optional codec shim.
include(${CMAKE_CURRENT_LIST_DIR}/spangap_requires.cmake)
idf_component_register(
    SRCS
        "audio.cpp"
        "codec.cpp"
        "wav.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        ${SPANGAP_REQUIRES}
        esp_driver_i2s
)
