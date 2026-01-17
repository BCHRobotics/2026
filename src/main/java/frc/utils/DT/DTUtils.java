/*
 * DTUtils.java
 * 
 * Drivetools Utility Class for FRC Robot Navigation System
 * 
 * This class provides a comprehensive set of string manipulation utilities designed specifically
 * for parsing and encoding navigation data structures (nav_field, nav_robot, nav_path, nav_marker).
 * It serves as a replacement for the Sys.cs utilities from the original Drivetools implementation.
 * 
 * Primary Use Cases:
 * - Parsing bracketed string encodings ({}, [], ()) of navigation objects
 * - Extracting variables from serialized data structures
 * - Finding matching bracket pairs in nested structures
 * - Splitting complex encoded strings into component parts
 * 
 * Design Notes:
 * - Uses String operations instead of char to maintain consistency (Java strings aren't directly indexable)
 * - All bracket-related functions handle three types: (), {}, []
 * - Most methods are static utilities that don't require instantiation
 * 
 * @see nav_field For field-level navigation structure
 * @see nav_robot For robot position and dimension encoding
 * @see nav_path For path waypoint encoding
 * @see nav_marker For navigation marker encoding
 */

package frc.utils.DT;

import java.util.LinkedList;
import java.util.List;

public class DTUtils {

    /**
     * Removes the first bracketed variable from an encoded string.
     * 
     * Used during deserialization to progressively parse multi-variable encodings.
     * For example, given "$vector3({x}{y}{z})", this removes "{x}" and returns the rest.
     * 
     * Process:
     * 1. Finds the closing bracket of the first variable (starting at index 0)
     * 2. Snips out everything after that closing bracket
     * 3. Returns the remaining string for further processing
     * 
     * @param input Encoded string containing bracketed variables
     * @return String with the first variable removed
     * 
     * Example:
     *   Input:  "{1}{2}{3}"
     *   Output: "{2}{3}"
     */
    public static String SubtractVariable(String input)
    {
        return SnipString(input, FindClosingBracket(input, 0) + 1, input.length() - 1);
    }

    /**
     * Extracts a substring from the input string using start and end indices (inclusive).
     * 
     * This custom implementation was likely created for consistency with the Drivetools C# codebase,
     * though it could be replaced with Java's built-in substring() method.
     * 
     * @param input The source string to extract from
     * @param dataStartIndex The starting index (inclusive)
     * @param dataEndIndex The ending index (inclusive)
     * @return The extracted substring
     * 
     * Note: This iterates character-by-character, which is less efficient than substring()
     *       but maintains compatibility with legacy code expectations.
     */
    public static String SnipString(String input, int dataStartIndex, int dataEndIndex)
    {
        String result = "";

        // Iterate through entire string, only adding characters within the specified range
        for (int i = 0; i < input.length(); i++)
        {
            if (i >= dataStartIndex && i <= dataEndIndex)
            {
                result += input.substring(i, i+1);
            }
        }

        return result;
    }

    /**
     * Finds the nth occurrence of a character in a string, starting from index 0.
     * 
     * Convenience overload that assumes search begins at the start of the string.
     * 
     * @param input The string to search within
     * @param toLookFor The single-character string to search for
     * @param numSkips Number of occurrences to skip before returning (0 = first occurrence)
     * @return The index of the target occurrence, or -1 if not found
     */
    public static int FindOccurance(String input, String toLookFor, int numSkips)
    {
        return FindOccurance(input, toLookFor, numSkips, 0);
    }

    /**
     * Finds the nth occurrence of a character in a string, starting from a specified index.
     * 
     * This is the core search function used throughout the parsing logic to locate specific
     * characters (like brackets, commas, etc.) in encoded strings.
     * 
     * IMPORTANT: Despite accepting a String parameter, toLookFor MUST be a single character.
     * This limitation exists due to Java's substring() usage instead of direct char indexing.
     * 
     * @param input The string to search within
     * @param toLookFor The single-character string to search for (e.g., "{", "[", ",")
     * @param numSkips Number of occurrences to skip before returning (0 = first occurrence)
     * @param startIndex Index to begin searching from
     * @return The index of the target occurrence, or -1 if not found
     * 
     * Warning: If numSkips is too large, the function will return -1 when no valid match is found.
     *          Always check for -1 return value to avoid index errors.
     */
    public static int FindOccurance(String input, String toLookFor, int numSkips, int startIndex)
    {
        // Iterate from startIndex to end of string
        for (int i = startIndex; i < input.length(); i++)
        {
            // Check if current character matches target
            if (input.substring(i, i + 1).equals(toLookFor))
            {
                // If we've skipped enough occurrences, return this index
                if (numSkips == 0)
                {
                    return i;
                }
                else
                {
                    numSkips--; // Otherwise, skip this occurrence
                }
            }
        }

        // Character not found after searching entire range
        return -1;
    }

    /**
     * Finds the matching closing bracket for an opening bracket at a given index.
     * 
     * This is a critical function for parsing nested data structures encoded as strings.
     * It handles three bracket types: (), {}, and [] with full nesting support.
     * 
     * Algorithm:
     * 1. Identifies the bracket type at startingIndex
     * 2. Tracks nesting level (starts at 1 for the opening bracket)
     * 3. Increments level for each nested opening bracket
     * 4. Decrements level for each closing bracket
     * 5. Returns index when level reaches 0 (matching closing bracket found)
     * 
     * Example:
     *   Input: "test{inner{nested}data}end", startingIndex=4 (first '{')
     *   Output: 23 (final '}')
     * 
     * @param input The string containing bracketed content
     * @param startingIndex Index of the opening bracket to match
     * @return Index of the matching closing bracket, or -1 if not found
     * 
     * Use Cases:
     * - Extracting complete encoded objects from complex strings
     * - Navigating nested navigation data structures
     * - Parsing multi-level path definitions
     */
    public static int FindClosingBracket(String input, int startingIndex)
    {
        // Determine bracket type from character at startingIndex
        String openChar = "(";
        String closeChar = ")";
        if (input.substring(startingIndex,startingIndex + 1).equals("(")) {  openChar = "("; closeChar = ")"; }
        if (input.substring(startingIndex, startingIndex + 1).equals("{")) { openChar = "{"; closeChar = "}"; }
        if (input.substring(startingIndex, startingIndex + 1).equals("[")) { openChar = "["; closeChar = "]"; }

        // Track nesting depth (1 = currently inside opening bracket)
        int bracketChildLevel = 1;

        // Search from position after opening bracket to end of string
        for (int i = startingIndex + 1; i < input.length(); i++)
        {
            // Nested opening bracket - go deeper
            if (input.substring(i,i + 1).equals(openChar))
            {
                bracketChildLevel++;
            }
            // Closing bracket - come back one level
            else if (input.substring(i,i + 1).equals(closeChar))
            {
                bracketChildLevel--;
            }

            // Found the matching closing bracket
            if (bracketChildLevel == 0)
            {
                return i;
            }
        }

        // No matching bracket found (malformed string)
        return -1;
    }

    /**
     * Splits a string into an array of bracketed sections.
     * 
     * This function extracts all complete bracketed sections (with matching open/close pairs)
     * from a string and returns them as an array. Each section INCLUDES its brackets.
     * 
     * Process:
     * 1. Scans the input string for opening brackets of the specified type
     * 2. For each opening bracket, finds its matching closing bracket
     * 3. Extracts the complete section (including brackets)
     * 4. Adds to result list and continues scanning after that section
     * 
     * Example Usage:
     *   Input:  "{value1}{value2}{value3}", splitCharacter="{"
     *   Output: ["{value1}", "{value2}", "{value3}"]
     * 
     * @param input The string to split (typically an encoded data structure)
     * @param splitCharacter The opening bracket type to split on: "(", "{", or "["
     * @return Array of strings, each containing a complete bracketed section
     * 
     * Note: Results INCLUDE the brackets themselves. This is useful for further parsing
     *       of nested structures where each section may need to be decoded separately.
     */
    public static String[] SplitString(String input, String splitCharacter)
    {   
        // Determine which opening bracket to search for
        String openChar = "(";
        if (splitCharacter.equals("(")) { openChar = "("; }
        if (splitCharacter.equals("[")) { openChar = "["; }
        if (splitCharacter.equals("{")) { openChar = "{"; }

        List<String> result = new LinkedList<String>();

        int startIndex = -1;  // Index of current section's opening bracket
        int endIndex = -1;    // Index of current section's closing bracket

        int limit = -1;       // Prevents re-processing already extracted sections

        // Scan through entire string looking for bracketed sections
        for (int i = 0; i < input.length(); i++)
        {
            // Found a new opening bracket beyond previously processed sections
            if (startIndex == -1 && input.substring(i, i+1).equals(openChar) && i > limit)
            {
                startIndex = i;
                endIndex = FindClosingBracket(input, startIndex);
            }

            // Have a complete section - extract it
            if (startIndex != -1 && endIndex != -1)
            {
                // Add complete section (including brackets) to result
                result.add(input.substring(startIndex, endIndex + 1));

                // Move limit past this section to avoid re-processing
                limit = endIndex;

                // Reset for next section
                startIndex = -1;
                endIndex = -1;
            }
        }

        // Convert LinkedList to array for return
        return result.toArray(new String[result.size()]);
    }
}
