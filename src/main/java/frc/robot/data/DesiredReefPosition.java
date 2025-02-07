package frc.robot.data;

import java.util.Objects;

public class DesiredReefPosition {

  final int face;
  final BranchLocation location;

  public DesiredReefPosition(int face, BranchLocation location) {
    this.face = face;
    this.location = location;
  }

  public int getFace() {
    return face;
  }

  public BranchLocation getLocation() {
    return location;
  }

  @Override
  public boolean equals(Object o) {
    if (o == this) {
      return true;
    }

    if (!(o instanceof DesiredReefPosition)) {
      return false;
    }

    return this.face == ((DesiredReefPosition) o).face
        && this.location == ((DesiredReefPosition) o).location;
  }

  @Override
  public int hashCode() {
    return Objects.hash(face, location);
  }
}
