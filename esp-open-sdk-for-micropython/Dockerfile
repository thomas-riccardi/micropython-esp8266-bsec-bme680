FROM larsks/esp-open-sdk

# rename our own built image
RUN sed -i 's:larsks/esp-open-sdk:esp-open-sdk:'  /usr/lib/esp-open-sdk/esp-open-sdk

# fix run wrapper on host ubuntu 20.04
RUN sed -i 's:/bin/sh:/bin/bash:' /usr/lib/esp-open-sdk/esp-open-sdk

# for http://docs.micropython.org/en/latest/develop/natmod.html
RUN pip3 --no-cache-dir install 'pyelftools>=0.25'
