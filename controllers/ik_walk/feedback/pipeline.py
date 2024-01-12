class Pipeline:

    def __init__(self, filters) -> None:
        self.filters = filters

    def handle(self, value):
        for filter in self.filters:
            value = filter.handle(value)

        return value