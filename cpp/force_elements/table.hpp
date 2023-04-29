#pragma once

#include "Simbody.h"
using namespace SimTK;

namespace LookupTable
{
    /**
     * @brief TableEntry data structure
     *
     */
    struct TableEntry
    {
        Real index;
        Real value;
        Real integ;
    };

    /**
     * @brief LookupTable1D with linear interpolation
     *
     */
    class LookupTable1D
    {
    protected:
        TableEntry m_table[48];
        int size = 0;
        bool m_allow_extrapolation_begin = true;
        bool m_allow_extrapolation_end = true;

    public:
        /**
         * @brief Construct a new Lookup Table 1 D object
         *
         * @param allow_extrapolation_begin
         * @param allow_extrapolation_end
         */
        LookupTable1D(bool allow_extrapolation_begin = true, bool allow_extrapolation_end = true)
        {
            m_allow_extrapolation_end = allow_extrapolation_begin;
            m_allow_extrapolation_end = allow_extrapolation_end;
        }

        /**
         * @brief Push new items to the table
         *
         * @param index
         * @param value
         * @return int
         */
        int push_back(Real index, Real value)
        {
            if (size >= sizeof(m_table))
                return 1;

            m_table[size++] = TableEntry{index, value, 0.0};

            // compute table integral
            for (auto idx = 1; idx < size - 1; idx++)
            {
                auto dx = (m_table[idx + 0].index - m_table[idx - 1].index);
                auto ym = (m_table[idx + 0].value + m_table[idx - 1].value) / 2.0;
                auto integ = ym * dx;
                m_table[idx + 1].integ = integ + m_table[idx + 0].integ;
            }

            // TODO: the vertical offset to get the potential energy needs to be computed
            // in order to do this, we would need to find the zero crossing of the value table
            // this is now assumed to be at (0,0) which is not necessarily true

            return 0;
        }

        /**
         * @brief Clear the table contents
         *
         * @return int
         */
        int clear()
        {
            size = 0;
            return 0;
        }

        /**
         * @brief Evaluate the lookup table with linear interpolation (extapolation configurable)
         *
         * @param index
         * @return Real
         */
        Real eval(Real index) const
        {
            if (size == 0)
                return 0.0;

            int idx;

            for (idx = 0; idx < size - 1; idx++)
            {
                if (m_table[idx + 1].index >= index)
                    break;
            }

            auto ratio = (index - m_table[idx + 0].index) / (m_table[idx + 1].index - m_table[idx + 0].index);

            if (!m_allow_extrapolation_begin)
                if (ratio < 0.0)
                    return m_table[idx + 0].value;

            if (!m_allow_extrapolation_end)
                if (ratio > 1.0)
                    return m_table[idx + 1].value;

            return m_table[idx + 0].value + ratio * (m_table[idx + 1].value - m_table[idx + 0].value);
        }

        /**
         * @brief Evaluate the integral of the lookup table (usefull for energy calculation)
         *
         * @param index
         * @return Real
         */
        Real eval_integ(Real index) const
        {
            if (size == 0)
                return 0.0;

            int idx;

            for (idx = 0; idx < size - 1; idx++)
            {
                if (m_table[idx + 1].index >= index)
                    break;
            }

            auto ratio = (index - m_table[idx + 0].index) / (m_table[idx + 1].index - m_table[idx + 0].index);

            if (!m_allow_extrapolation_begin)
                if (ratio < 0.0)
                    return m_table[idx + 0].integ;

            if (!m_allow_extrapolation_end)
                if (ratio > 1.0)
                    return m_table[idx + 1].integ;

            return m_table[idx + 0].integ + ratio * (m_table[idx + 1].integ - m_table[idx + 0].integ);
        }

        /**
         * @brief Get the size object
         *
         * @return int
         */
        int get_size()
        {
            return size;
        }
    };

    /*
        class LookupTable1DVar
        {
        private:
            std::vector<TableEntry> table;
            bool m_allow_extrapolation_begin = true;
            bool m_allow_extrapolation_end = true;

        public:
            LookupTable1DVar(bool allow_extrapolation_begin = true, bool allow_extrapolation_end = true)
            {
                m_allow_extrapolation_end = allow_extrapolation_begin;
                m_allow_extrapolation_end = allow_extrapolation_end;
            }

            int push_back(Real index, Real value)
            {
                table.push_back(TableEntry{index, value});
                return 0;
            }

            int clear()
            {
                table.clear();
                return 0;
            }

            Real eval(Real index) const
            {
                if (table.size() == 0)
                    return 0.0;

                int idx;

                for (idx = 0; idx < table.size() - 1; idx++)
                {
                    if (table[idx + 1].index >= index)
                        break;
                }

                auto ratio = (index - table[idx + 0].index) / (table[idx + 1].index - table[idx + 0].index);

                if (!m_allow_extrapolation_begin)
                    if (ratio < 0.0)
                        return table[idx + 0].value;

                if (!m_allow_extrapolation_end)
                    if (ratio > 1.0)
                        return table[idx + 1].value;

                return table[idx + 0].value + ratio * (table[idx + 1].value - table[idx + 0].value);
            }
        };
        */
}
