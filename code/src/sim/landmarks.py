import numpy as np

class Landmarks():
    def __init__(self, nl, xmin=0, xmax=50, ymin=0, ymax=50, zmin=-50, zmax=50, type="UNIFORM"):
        if ymax == 0:
            ymax = 1
        if xmax == 0:
            xmax = 1
        if zmax == 0:
            zmax = 1

        if type == "UNIFORM":
            xs = np.random.uniform(xmin,xmax,size=nl)
            ys = np.random.uniform(ymin,ymax,size=nl)
            zs = np.random.uniform(zmin,zmax,size=nl)
        elif type == "CLUSTERS":
            n_clusters = 7
            avg_std_xy = 1.0
            std_std_xy = avg_std_xy/3.0
            std_xy = np.random.normal(avg_std_xy, std_std_xy, n_clusters)
            sizes = np.random.multinomial(nl, [1/n_clusters]*n_clusters)

            xs, ys, zs = [], [], []

            for k in range(n_clusters):
                cx = np.random.uniform(xmin + avg_std_xy, xmax - avg_std_xy)
                cy = np.random.uniform(ymin + avg_std_xy, ymax - avg_std_xy)

                xs.append(np.random.normal(cx, std_xy[k], sizes[k]))
                ys.append(np.random.normal(cy, std_xy[k], sizes[k]))

            xs = np.concatenate(xs)
            ys = np.concatenate(ys)
            zs = np.random.uniform(zmin,zmax,size=nl)

        self.nl = nl
        self.landmarks = {}
        for i in range(nl):
            self.landmarks[i] = np.array([xs[i], ys[i], zs[i]])

    @classmethod
    def from_run(cls, run, z):
        """Use the planner's landmark field instead of a random one.

        The plan is 2D, so the landmarks are lifted onto a single depth plane.
        `detect_landmarks` derives elevation from dz, so this choice does change
        the elevation measurements: putting the field at the primary's own depth
        keeps its elevations near zero and lets the depth-staggered supports see
        slightly different angles, which is realistic but is an assumption the
        planner's own model does not make.
        """
        self = cls.__new__(cls)
        self.nl = len(run.landmarks)
        self.landmarks = {i: np.array([x, y, z]) for i, (x, y) in enumerate(run.landmarks)}
        return self

    def detect_landmarks(self, agent_pose, std, add_noise=False, max_range=100):
        detected_landmarks = {}
        for i in range(self.nl):
            trans_diff = agent_pose[:3] - self.landmarks[i]
            r = np.linalg.norm(trans_diff)

            if r < max_range:
                phi = np.arctan2(trans_diff[1],trans_diff[0])# - agent_pose[2]   # azimuth
                theta = np.arcsin(trans_diff[2]/r)                              # elevation
                if add_noise:
                    
                    phi += np.random.normal(0, std[0])
                    theta += np.random.normal(0, std[1])
                    r += np.random.normal(0, std[2])

                detected_landmarks[i] = (phi, theta, r)

        return detected_landmarks

