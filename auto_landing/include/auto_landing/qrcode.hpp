#include <cv.h>

namespace cv {
class CV_EXPORTS_W QRCodeDetector
{
public:
    CV_WRAP QRCodeDetector();
    ~QRCodeDetector();

    /** @brief sets the epsilon used during the horizontal scan of QR code stop marker detection.
     @param epsX Epsilon neighborhood, which allows you to determine the horizontal pattern
     of the scheme 1:1:3:1:1 according to QR code standard.
    */
    CV_WRAP void setEpsX(double epsX);
    /** @brief sets the epsilon used during the vertical scan of QR code stop marker detection.
     @param epsY Epsilon neighborhood, which allows you to determine the vertical pattern
     of the scheme 1:1:3:1:1 according to QR code standard.
     */
    CV_WRAP void setEpsY(double epsY);

    /** @brief Detects QR code in image and returns the quadrangle containing the code.
     @param img grayscale or color (BGR) image containing (or not) QR code.
     @param points Output vector of vertices of the minimum-area quadrangle containing the code.
     */
    CV_WRAP bool detect(InputArray img, OutputArray points) const;

    /** @brief Decodes QR code in image once it's found by the detect() method.
     Returns UTF8-encoded output string or empty string if the code cannot be decoded.
     @param img grayscale or color (BGR) image containing QR code.
     @param points Quadrangle vertices found by detect() method (or some other algorithm).
     @param straight_qrcode The optional output image containing rectified and binarized QR code
     */
    CV_WRAP std::string decode(InputArray img, InputArray points, OutputArray straight_qrcode = noArray());

    /** @brief Both detects and decodes QR code
     @param img grayscale or color (BGR) image containing QR code.
     @param points opiotnal output array of vertices of the found QR code quadrangle. Will be empty if not found.
     @param straight_qrcode The optional output image containing rectified and binarized QR code
     */
    CV_WRAP std::string detectAndDecode(InputArray img, OutputArray points=noArray(),
                                        OutputArray straight_qrcode = noArray());
protected:
    struct Impl;
    Ptr<Impl> p;
};
}