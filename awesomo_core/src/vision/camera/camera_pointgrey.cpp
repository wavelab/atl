
int Camera::initFirefly() {
  FlyCapture2::Error error;
  FlyCapture2::Property property;
  FlyCapture2::Property gain_property;

  // setup
  this->capture_firefly = new FlyCapture2::Camera();

  // connect
  error = this->capture_firefly->Connect(0);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to connect to camera!\n");
    return -1;
  } else {
    printf("Firefly camera connected!\n");
  }

  // set video mode format and frame rate
  error = this->capture_firefly->SetVideoModeAndFrameRate(
    FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_60);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to set camera video mode and frame rate!\n");
    return -1;
  } else {
    printf("Firefly camera video mode and frame rate configured!\n");
  }

  // configure exposure
  property.type = FlyCapture2::AUTO_EXPOSURE;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_exposure_value;  // exposure value

  error = this->capture_firefly->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to configure camera exposure!\n");
    return -1;
  } else {
    printf("Firefly camera exposure configured!\n");
  }

  // configure gain
  property.type = FlyCapture2::GAIN;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_gain_value;  // set the gain

  error = this->capture_firefly->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed to configure camera gain!\n");
    return -1;
  } else {
    printf("Firefly camera gain configured!\n");
  }

  // start camera
  error = this->capture_firefly->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("ERROR! Failed start camera!\n");
    return -1;
  } else {
    printf("Firefly initialized!\n");
  }

  return 0;
}

int Camera::getFramePointGrey(cv::Mat &image) {
  double data_size;
  double data_rows;
  unsigned int row_bytes;

  FlyCapture2::Image raw_img;
  FlyCapture2::Image rgb_img;
  FlyCapture2::Error error;

  error = this->capture_firefly->RetrieveBuffer(&raw_img);
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("Video capture error!\n");
    return -1;
  }

  // convert to rgb
  raw_img.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_img);

  // convert to opencv mat
  data_size = rgb_img.GetReceivedDataSize();
  data_rows = rgb_img.GetRows();
  row_bytes = data_size / data_rows;
  cv::Mat(rgb_img.GetRows(),
          rgb_img.GetCols(),
          CV_8UC3,
          rgb_img.GetData(),
          row_bytes)
    .copyTo(image);

  // resize the image to reflect camera mode
  cv::resize(image,
             image,
             cv::Size(this->config->image_width, this->config->image_height));

  return 0;
}
