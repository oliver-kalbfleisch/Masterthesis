package base;

import org.joml.Vector3f;

import au.edu.federation.utils.Vec3f;

/**
 * 
 * @author oliver 
 * Utility Class for storing tracking data
 */
public class HandData {

	private int numTrackers;
	private Vec3f[] currentTrackerPos;
	private Vector3f handBasePos;

	public HandData(int numTrackers) {
		this.numTrackers = numTrackers;
		this.currentTrackerPos = new Vec3f[numTrackers];
		this.setHandBasePos(new Vector3f());
	}

	/**
	 * Function calculates position data in relation to initialy calibrated origin
	 * position
	 * 
	 * @param fingerNum
	 *            number of finger to update (thumb = 0)
	 * @param updatePos
	 *            the new position from the trackng data
	 * @return the new calculated 3d postion
	 */
	public Vec3f updateCurrentFingerPos(int fingerNum, Vec3f updatePos) {
		currentTrackerPos[fingerNum] = updatePos;
		return updatePos;

	}

	public int getNumTrackedFingers() {
		return numTrackers;
	}

	public void setNumTrackedFingers(int numTrackedFingers) {
		this.numTrackers = numTrackedFingers;
	}

	public Vec3f[] getCurrentFingerPos() {
		return currentTrackerPos;
	}

	public void setCurrentFingerPos(Vec3f[] currentFingerPos) {
		this.currentTrackerPos = currentFingerPos;
	}

	public Vector3f getHandBasePos() {
		return handBasePos;
	}

	public void setHandBasePos(Vector3f handBasePos) {
		this.handBasePos = handBasePos;
	}

}
