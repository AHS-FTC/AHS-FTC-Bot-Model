package edu.ahs.robotics.util.ftc;

public class Toggle extends Switch {
    private boolean enabled = false;

    public Toggle() {
        super();
    }

    public boolean canFlip() {
        if (super.canFlip()) {
            enabled = !enabled;
            return true;
        }
        return false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
