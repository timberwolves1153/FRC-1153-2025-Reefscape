/*

Hello!  This file is just to realize what our Unit tests need to be for the Windmill!
 - This will be updated with the correct values as we go along
 - This will be used to note the tests the Windmill subsystem
 - Keep in mind that this is a work in progress and will be updated as we go along through the season
 - The notes are for the UNIT TESTS, so it will be specfically for the limit switches, and allat jazz
 - We need to have the:
        -import org.junit.Test;
        -import static org.junit.Assert.assertEquals;
        -import static org.junit.Assert.assertTrue;
        -import static org.junit.Assert.assertFalse;

Tests:
    1. Limit Switches
        - Probably a button Limit switch
            - Purpose: You wanna make sure we dont spin so much we create a black hole!!! (or break the robot potato potato)
            - How: It would need ot be placed in the place right past where we need to collect the agale and then right past after we collect the coral
               - Basically, it needs to be "Double Sided" so it can be used for both the coral and the agale
                    - Sample code:
                        - public void testLimitSwitch() {
                            windmill.setTargetPosition(30);
                            assertEquals(30, windmill.getAngle()); //determine fale true
                            windmill.setTargetPosition(90);
                            assertEquals(90, windmill.getAngle()); // determine false true
                            windmill.setTargetPosition(180);
                            assertEquals(180, windmill.getAngle()); // determine false true
                            windmill.setTargetPosition(270);
                            assertEquals(270, windmill.getAngle()); // determine false true
                            windmill.setTargetPosition(360);
                            assertEquals(360, windmill.getAngle()); // determine false true
                        }
                            - This is just a simple test to make sure the limit switches are working, by trying to run past, and we make sure that the ones past hte limit switch return false
            - Placement: Just an idea, we place it on the top, like a V, and it will work pretty well

    2. Code checks
        - Angles
            - Purpose: Make sure the angles are correct and arent off the line
                - Also don't want to make a black hole just in case of limit switch faulure
                - How: just do a simple base code like
                    - public void testAngles() {
                        windmill.setTargetPosition(30);
                        assertEquals(30, windmill.getAngle());
                    }
                         - This is just a simple test to make sure the angles are correct

        - Volts
            - Purpose: Make sure we are not burning the motors
                - Also, we wanna find a way to hold!
            - How: Just a simple test like
                - public void testVolts() {
                    windmill.setVoltage(12);
                    assertEquals(12, windmill.getVoltage());
                }
                    - This is just a simple test to make sure the volts are correct

        - Hold
            - Purpose: So we can score the stuff, without a sliding windmill
            - How: Just a simple test like
                - public void testHold() {
                    windmill.holdPosition();
                    assertEquals(windmill.getAngle(), windmill.getAngle());
                }
                    - This is just a simple test to make sure the hold is working




 */
