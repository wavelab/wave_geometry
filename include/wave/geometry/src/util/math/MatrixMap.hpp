/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_MATRIXMAP_HPP
#define WAVE_GEOMETRY_MATRIXMAP_HPP

#include <Eigen/Core>
#include <boost/container/flat_map.hpp>

namespace wave {

/**
 * Stores a collection of equal-height matrices in one contiguous matrix.
 *
 * Stores matrices of size n*m_1, n*m_2, ... n*m_p,  where n and
 * all m_i are known at construct time.
 *
 * Indexing is done through a corresponding set of keys k_1, k_2, ... k_p.
 *
 * MatrixMap is constructed with a map of keys k_i to widths m_i, and its size cannot not
 * change later. The matrix is initially resized but not initialized (holds garbage).
 *
 * @tparam Key the key type used for indexing
 * @tparam Scalar the scalar type of the matrix (e.g. double)
 */
template <typename Key, typename Scalar>
class MatrixMap {
    struct IndexPair {
        int col_index;
        int col_width;
    };

    using IndexMap = boost::container::flat_map<Key, IndexPair>;

 public:
    /** Constructs and resizes the matrix. (Doesn't initialize. It holds garbage).
     *
     * Variant for dynamic-height MatrixMap.
     *
     * @param begin, end a pair of iterators to a sequence of type <Key, int> holding
     * the {key, column width} for each matrix. For best performance it should be sorted;
     * duplicates allowed.
     * @param number of rows
     */
    template <typename MapIt>
    MatrixMap(const MapIt &begin, const MapIt &end, Eigen::Index rows) {
        int col = 0;
        index_map.reserve(std::distance(begin, end));
        for (auto it = begin; it != end; ++it) {
            const auto &key = it->first;
            const auto &width = it->second;
            index_map.emplace_hint(index_map.end(), key, IndexPair{col, width});
            col += width;
        }
        storage.resize(rows, col);
    }

    /** Returns a block representing the matrix for the given key */
    auto operator[](const Key &key) {
        const auto &v = this->index_map[key];
        return this->storage.block(0, v.col_index, storage.rows(), v.col_width);
    }

    /** Returns a const block representing the matrix for the given key */
    auto operator[](const Key &key) const {
        const auto &v = this->index_map[key];
        return this->storage.block(0, v.col_index, storage.rows(), v.col_width);
    }

    /** Returns a block representing the matrix for the given key
     * @throws out_of_range if key is not present
     */
    auto at(const Key &key) {
        const auto &v = this->index_map.at(key);
        return this->storage.block(0, v.col_index, storage.rows(), v.col_width);
    }

    /** Returns a const block representing the matrix for the given key
     * @throws out_of_range if key is not present
     */
    auto at(const Key &key) const {
        const auto &v = this->index_map.at(key);
        return this->storage.block(0, v.col_index, storage.rows(), v.col_width);
    }

    /** Returns 1 if the key is present, 0 otherwise */
    auto count(const Key &key) const {
        return this->index_map.count(key);
    }

    /** Set all blocks to zero */
    void setZero() {
        this->storage.setZero();
    }


 private:
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> storage;
    // Map of key -> (column index, column width) index
    IndexMap index_map;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_MATRIXMAP_HPP
