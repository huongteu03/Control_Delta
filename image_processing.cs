    public static ResultObject FindObject(Mat img, List<ObjectClassify> objects)
        {
            // Tạo kết quả trả về
            ResultObject result = new ResultObject(false);
            double maxObjSize = 1000;

            // Chuyển đổi sang kênh màu HSV
            Mat imgHSV = new Mat();
            Cv2.CvtColor(img, imgHSV, ColorConversionCodes.BGR2HSV);

            // Tìm từng object trên ảnh 
            for (int i = 0; i < objects.Count; i++)
            {
                // Tạo ảnh mask đánh dấu object
                Mat mask = new Mat();

                // Nếu dải giá trị H: min < max
                if (objects[i].hmin < objects[i].hmax) 
                {
                    Scalar lower = new Scalar(objects[i].hmin, objects[i].smin, objects[i].vmin);
                    Scalar upper = new Scalar(objects[i].hmax, objects[i].smax, objects[i].vmax);
                    Cv2.InRange(imgHSV, lower, upper, mask);
                }
                // nếu không, max < min
                else
                {
                    Mat mask1 = new Mat();
                    Mat mask2 = new Mat();

                    Scalar lower1 = new Scalar(objects[i].hmin, objects[i].smin, objects[i].vmin);
                    Scalar upper1 = new Scalar(179, objects[i].smax, objects[i].vmax);
                    Cv2.InRange(imgHSV, lower1, upper1, mask1);

                    Scalar lower2 = new Scalar(0, objects[i].smin, objects[i].vmin);
                    Scalar upper2 = new Scalar(objects[i].hmax, objects[i].smax, objects[i].vmax);
                    Cv2.InRange(imgHSV, lower2, upper2, mask2);

                    Cv2.BitwiseOr(mask1, mask2, mask);
                }

                Cv2.ImShow("Mask", mask);
                Cv2.ImWrite("C:\\Users\\Admin\\Desktop\\mask.png", mask);

                // Xử lý hình thái
                Mat kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new OpenCvSharp.Size(3, 3));

                Mat morph1 = new Mat();
                Mat morph2 = new Mat();
                Mat morph3 = new Mat();

                Cv2.Dilate(mask, morph1, kernel, iterations: 2);
                Cv2.Erode(morph1, morph2, kernel, iterations: 6);
                Cv2.Dilate(morph2, morph3, kernel, iterations: 4);

                Cv2.ImShow("Morphology", morph3);
                Cv2.ImWrite("C:\\Users\\Admin\\Desktop\\morphology.png", morph3);

                // Tìm contours
                OpenCvSharp.Point[][] contours;
                HierarchyIndex[] hierarchy;
                Cv2.FindContours(morph3, out contours, out hierarchy, RetrievalModes.List, ContourApproximationModes.ApproxNone);

                if (contours.Length > 0)
                {
                    // Tìm contour lớn nhất và lớn hơn ? pixel
                    double maxSize = maxObjSize;
                    int iBestContour = -1;
                    for (int j = 0; j < contours.Length; ++j)
                    {
                        double area = Cv2.ContourArea(contours[j]);
                        if (area > maxSize)
                        {
                            iBestContour = j;
                            maxSize = area;
                        }
                    }

                    if((iBestContour != -1) && (maxSize > maxObjSize))
                    {
                        maxObjSize = maxSize;
                        RotatedRect RRect = Cv2.MinAreaRect(contours[iBestContour]);
                        result = new ResultObject(true, objects[i].Type, RRect);
                    }
                }
            }

            // Dispose
            //imgHSV.Dispose();

            return result;
        }