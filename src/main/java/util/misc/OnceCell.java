package util.misc;

import java.util.Optional;
import java.util.function.Supplier;

public class OnceCell<T> {
    Optional<T> value;

    public OnceCell() {
        value = Optional.empty();
    }

    public T getOrInitialize(Supplier<T> supplier) {
        if (value.isEmpty()) {
            value = Optional.of(supplier.get());
        }
        return value.get();
    }
}
