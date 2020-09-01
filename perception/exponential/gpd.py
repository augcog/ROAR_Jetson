from perception.util import get_roll_stats, roll_frame
import exponential_model as exp_model
import leastsq_exp as lsq_model
import numpy as np

class gpd_sensor():
    
    def __init__(self, depth, t=0.089, del_ang=0.02, fit_type='exp'):
        self.thresh = t
        self.fit_type = fit_type
        self.del_ang = del_ang
        self.roll_ang = 0
        self.rot_axis = [0, 0, 1]
        
        self.orig_preds = self.gpd_mesh(depth)
        self.preds = np.copy(self.orig_preds)
        
    def output_gpd(self, d_frame):
        diff = np.abs(d_frame - self.preds)
        dets = diff > self.thresh
        orig_d_frame = np.copy(d_frame)
        d_frame[dets > 0] = 0
        
        try:
            new_roll_ang, self.rot_axis = get_roll_stats(d_frame)
            if np.abs(self.roll_ang - new_roll_ang) > self.del_ang :
                self.roll_ang = new_roll_ang
                self.preds = roll_frame(self.orig_preds, self.roll_ang, -1*self.rot_axis)
        except Exception as e:
            print("ERROR MESSAGE:", e)
        
        return orig_d_frame, d_frame, dets
    
    def gpd_mesh(self, depth_image):
        xs = []
        data = []
        max_depth = 0
        for i in range(depth_image.shape[0] - 1, -1, -1):
            j = np.argmax(depth_image[i,:])
            d = depth_image[i][j]
            if d > 0.3:
                break
            if d > max_depth and d > 0.01:
                max_depth = d
                xs.append(i)
                data.append(d)

        xs = np.array(xs[::-1], dtype=np.float64)
        data = np.array(data[::-1], dtype=np.float64)

        if self.fit_type == 'lsq':
            a, b, c, d = lsq_model.fit(xs / xs.max(), data)
            pred_func = lsq_model.construct_f(a, b, c, d)
            rows = np.meshgrid(
                np.arange(depth_image.shape[1]), np.arange(depth_image.shape[0])
            )[1]
            preds = pred_func(rows / rows.max())
            preds[preds > 1] = 0
            return preds
        else:
            a, b, c ,p, q = exp_model.fit(xs, data)
            pred_func = exp_model.construct_f(a, b, c, p, q)
            rows = np.meshgrid(
                np.arange(depth_image.shape[1]), np.arange(depth_image.shape[0])
            )[1]
            preds = pred_func(rows)
            preds[preds > 1] = 0
            return preds