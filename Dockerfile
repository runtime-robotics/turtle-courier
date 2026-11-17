FROM ghcr.io/ssskiv/virtual-courier-scene@sha256:98246581b07266666fd29c3a2fd4e55a4864ba40bca3f36d680c59baa87ec44e
RUN apt-get update -o Acquire::Retries=3 \
 && apt-get install -y --no-install-recommends python3-opencv \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY . /app
ENV ROS_DOMAIN_ID=57 ROS_LOCALHOST_ONLY=1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 PYTHONUNBUFFERED=1
ENTRYPOINT ["/app/run.sh"]
