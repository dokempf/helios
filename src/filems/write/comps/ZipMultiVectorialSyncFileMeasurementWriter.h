#pragma once

#include <filems/write/comps/ZipMultiSyncFileWriter.h>

#include <memory>
#include <vector>

namespace helios { namespace filems{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Concrete class specializing ZipMultiSyncFileWriter to write a vector
 *  of measurements to multiple zip files
 *
 * @see filems::ZipMultiSyncFileWriter
 * @see filems::ZipMeasurementWriteStrategy
 * @see Measurement
 * @see filems::ZipSyncFileMeasurementWriter
 */
class ZipMultiVectorialSyncFileMeasurementWriter :
    public ZipMultiSyncFileWriter<
        vector<Measurement> const &,
        glm::dvec3 const &
    >
{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief The measurement write strategies that are wrapped by the main
     *  write strategies in a vectorial fashion
     *  ( filems::ZipMultiSyncFileWriter::writeStrategy )
     * @see filems::ZipMeasurementWriteStrategy
     */
    std::vector<ZipMeasurementWriteStrategy> zmws;

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief ZIP synchronous file measurement vector multi-writer constructor
     * @see filems::ZipMultiSyncFileWriter::ZipMultiSyncFileWriter
     */
    explicit ZipMultiVectorialSyncFileMeasurementWriter(
        vector<string> const &path,
        int compressionMode = boost::iostreams::zlib::best_compression
    ) :
        ZipMultiSyncFileWriter<
            vector<Measurement> const &,
            glm::dvec3 const &
        >(path, compressionMode)
    {
        // Build measurement write strategies
        buildMeasurementWriteStrategies();
        // Build vectorial write strategies
        // WARNING : It must be done after building the measurement write
        // strategies to be wrapped by the vectorial strategy. If the vector
        // of measurements strategies is modified, then the references in the
        // vectorial strategy objects will be inconsistent
        buildVectorialWriteStrategies();
    }
    virtual ~ZipMultiVectorialSyncFileMeasurementWriter() = default;

    // ***   B U I L D   *** //
    // ********************* //
    /**
     * @brief Build the measurement write strategies for the zip multi
     *  vectorial synchronous file measurement writer
     */
     void buildMeasurementWriteStrategies(){
        // Build measurement write strategies
        size_t const nStreams = path.size();
        for(size_t i = 0 ; i < nStreams ; ++i){
            zmws.emplace_back(this->ofs[i], *(this->oa[i]));
        }
     }
    /**
     * @brief Build the vectorial write strategies for the zip multi vectorial
     *  synchronous file measurement writer.
     *
     * <b>WARNING!</b> calling the buildMeasurementWriteStrategies() method
     *  will invalidate the state generated by buildVectorialWriteStrategies()
     */
    void buildVectorialWriteStrategies(){
        // Build vectorial write strategies
        size_t const nStreams = path.size();
        for(size_t i = 0 ; i < nStreams ; ++i){
            this->writeStrategy.push_back(make_shared<VectorialWriteStrategy<
                Measurement,
                glm::dvec3 const &
            >>(zmws[i]));
        }
    }

    // ***  W R I T E  *** //
    // ******************* //
    /**
     * @see MultiSyncFileWriter::indexFromWriteArgs
     */
    size_t indexFromWriteArgs(
        vector<Measurement> const &measurements,
        glm::dvec3 const &offset
    ) override {return measurements[0].devIdx;}

};

}}