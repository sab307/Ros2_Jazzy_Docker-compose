FROM eclipse/zenoh:nightly

# Install basic tools for debugging and healthchecks
# - bash easier scripting
# - curl HTTP queries
# netcat-openbsd (nc) TCP port check
#FROM alpine:3.18
#RUN apk update
#RUN apk add bash 
#RUN apk add curl 
#RUN apk add netcat-openbsd 
#FROM alpine:3.18
#RUN echo "http://dl-0.alpinelinux.org/alpine/v3.18/main" > /etc/apk/repositories \
#RUN cat /etc/os-release
RUN apk update \
  && apk add --no-cache bash curl netcat-openbsd

# Default command: start the Zenoh router
# CMD [ "zenohd" ]
#FROM eclipse/zenoh:nightly