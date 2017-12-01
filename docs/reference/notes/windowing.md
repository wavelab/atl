As noted in the work of \cite{Ling2014} the AprilTag library's performance can
be further increased by reducing the brightness of the image such that a
majority of the image is black. Building upon this idea whenever a AprilTag is
detected its location in the image space is used to create a mask which blacks
out the majority of the next image except for the expected location of the
AprilTag in the image. This method is performed as follows, once the center of
the AprilTag in camera frame is found the top left and bottom right corners of
the AprilTag can be calculated with:

\begin{align}
  X_{\text{top left}} &= X - (l / 2) - X_{\text{padding}} \\
  Y_{\text{top left}} &= Y - (l / 2) - Y_{\text{padding}} \\
  X_{\text{bottom right}} &= X + (l / 2) + X_{\text{padding}} \\
  Y_{\text{bottom right}} &= Y + (l / 2) + Y_{\text{padding}}
\end{align}

Where $X, Y$ are positions of the AprilTag in the camera frame,
$X_{\text{padding}}, Y_{\text{padding}}$ are the mask padding in the camera
frame, $l$ is the AprilTag side length in meters, and finally $X_{\text{top
left}}, Y_{\text{top left}}, X_{\text{bottom right}}, Y_{\text{bottom right}}$
represent the top left and bottom right corners in the camera frame. Adding the
mask padding at this point allows the padding to be defined in the camera frame
which increases and decreases proportionally in the image frame depending on
the depth-distance between camera and AprilTag.

Using the pin-hole camera model, the AprilTag's corners can be converted from
camera frame into image frame as follows:

\begin{align}
  x = \dfrac{f_{x} \times X}{Z} \qquad \qquad
  y = \dfrac{f_{y} \times Y}{Z}
\end{align}

Where $f_{x}$ and $f_{y}$ are the focal length in pixels in the $x$ and $y$
axis, and finally $x, y$ are image pixel coordinates. Using the image
coordinates of the top left and bottom right corners of the AprilTag a mask can
be created.
