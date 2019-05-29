
  
    public class Shield
{
    //the 28 combinations
    static public string[] LABELS = {
        "0-1", "0-2", "0-3", "0-4", "0-5", "0-6", "0-7", "1-2", "1-3", "1-4",
        "1-5", "1-6", "1-7", "2-3", "2-4", "2-5", "2-6", "2-7", "3-4", "3-5",
        "3-6", "3-7", "4-5", "4-6", "4-7", "5-6", "5-7", "6-7" }; 
	
	/**
	 * Indices of values in a measurement per electrode, which can be used
	 * to retrieve information per electrode. This map is only relevant when
	 * using the Skweezee shield for Arduino.
	 * 
	 * SUBVECTOR_MAP[0] contains all indices of values in a measurement
	 * involving the first electrode 0, so pointing to the measured values
	 * {index_of_0-1, index_of_0-2, ..., index_of_0-7}.
	 * 
	 * SUBVECTOR_MAP contains 8x7 indices from 0 to 27.
	 * 
	 * Note that accessing the values in this way introduces a duplication of
	 * data. Each value can be accessed twice: for example SUBVECTOR_MAP[3][4]
	 * and SUBVECTOR_MAP[5][3] contain the index of the same value (the index
	 * of pair {3-5}: 20) 
	 */
	static public int[,] SUBVECTOR_MAP = {
        {0, 1, 2, 3, 4, 5, 6}, {0, 7, 8, 9, 10, 11, 12}, {1, 7, 13, 14, 15, 16, 17}, 
        {2, 8, 13, 18, 19, 20, 21}, {3, 9, 14, 18, 22, 23, 24}, {4, 10, 15, 19, 22, 25, 26},
        {5, 11, 16, 20, 23, 25, 27}, {6, 12, 17, 21, 24, 26, 27}};
       
}
