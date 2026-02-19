package frc.utils.dto;

import java.util.LinkedList;
import java.util.List;

// kind of a replacement for Sys.cs on the Drivetools side
// in other words, a whole lotta helper functions and most of them are String-related

// a note on some of the string processing functions:
// I'm using strings where I could be using chars, the reason is that strings in java cannot be indexed,
// so instead of just doing string[0] and getting a char i need to use substring() and get a string
// can't compare char and string directly, so may as well just always use strings
// ^^ it's dumb, i know

public class DTUtils {

    // given a string that's made up of sets of {} (like a vector, for example: $vector3({x}{y}{z})),
    // this function will remove the first set of {}
    // this is used when decoding functions from strings
    public static String SubtractVariable(String input)
    {
        return SnipString(input, FindClosingBracket(input, 0) + 1, input.length() - 1);
    }

    // takes a section out of a string
    // I have no idea why I didn't just use subString()? maybe im stupid
    public static String SnipString(String input, int dataStartIndex, int dataEndIndex)
    {
        String result = "";

        for (int i = 0; i < input.length(); i++)
        {
            if (i >= dataStartIndex && i <= dataEndIndex)
            {
                result += input.substring(i, i+1);
            }
        }

        return result;
    }

    // same as below, but the start index is not provided and assumed to be 0
    public static int FindOccurance(String input, String toLookFor, int numSkips)
    {
        return FindOccurance(input, toLookFor, numSkips, 0);
    }

    // starting at a given index, step through the string until a character is found
    // NOTE: even though you pass it in as a string, IT HAS TO BE A SINGLE CHARACTER!! (looking for substrings won't work)

    // oh, and if you're not looking for the first occurence, 
    // then instead of passing 0 for numSkips you can pass a number and it'll skip that many times before returning
    // (of course be weary of skipping too much and not finding a valid match, that'll just get you a -1)
    public static int FindOccurance(String input, String toLookFor, int numSkips, int startIndex)
    {
        for (int i = startIndex; i < input.length(); i++)
        {
            if (input.substring(i, i + 1).equals(toLookFor))
            {
                
                if (numSkips == 0)
                {
                    return i;
                }
                else
                {
                    numSkips--;
                }
            }
        }

        return -1;
    }

    // pass in the index of a bracket (any type: (, {, [  ) and it will return the index of the bracket that closes it
    // INCREDIBLY useful when separating complicated classes that have been encoded to a string
    public static int FindClosingBracket(String input, int startingIndex)
    {
        String openChar = "(";
        String closeChar = ")";
        // System.out.println(input.substring(startingIndex,startingIndex + 1) + "AAA");
        if (input.substring(startingIndex,startingIndex + 1).equals("(")) {  openChar = "("; closeChar = ")"; }
        if (input.substring(startingIndex, startingIndex + 1).equals("{")) { openChar = "{"; closeChar = "}"; }
        if (input.substring(startingIndex, startingIndex + 1).equals("[")) { openChar = "["; closeChar = "]"; }

        int bracketChildLevel = 1;

        for (int i = startingIndex + 1; i < input.length(); i++)
        {
            // System.out.println(closeChar);
            if (input.substring(i,i + 1).equals(openChar))
            {
                bracketChildLevel++;
            }
            else if (input.substring(i,i + 1).equals(closeChar))
            {
                // System.out.println("MINUS");
                bracketChildLevel--;
            }

            if (bracketChildLevel == 0)
            {
                return i;
            }
        }

        return -1;
    }

    // split a given string into arrays, each resulting section separated by a character
    // iirc the results DO NOT INCLUDE THE CHAR ITSELF
    public static String[] SplitString(String input, String splitCharacter)
    {   
        // prob could have gotten away with chars here but hey, consistency
        String openChar = "(";
        if (splitCharacter.equals("(")) { openChar = "("; }
        if (splitCharacter.equals("[")) { openChar = "["; }
        if (splitCharacter.equals("{")) { openChar = "{"; }

        List<String> result = new LinkedList<String>();

        int startIndex = -1;
        int endIndex = -1;

        int limit = -1;

        for (int i = 0; i < input.length(); i++)
        {
            if (startIndex == -1 && input.substring(i, i+1).equals(openChar) && i > limit) // looking for the character that starts a split range
            {
                startIndex = i;
                endIndex = FindClosingBracket(input, startIndex);
            }

            if (startIndex != -1 && endIndex != -1)
            {
                result.add(input.substring(startIndex, endIndex + 1));

                limit = endIndex;

                startIndex = -1;
                endIndex = -1;
            }
        }

        return result.toArray(new String[result.size()]);
    }
}
