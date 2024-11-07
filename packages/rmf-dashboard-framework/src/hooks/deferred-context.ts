import React from 'react';

/**
 * Creates a context that is initially `null` but is non-nullable when using the value.
 * A value MUST be provided before any consumer uses it.
 *
 * The use of this is to avoid non-null assertions when you know the context is always
 * available but a default value cannot be provided statically.
 */
export function createDeferredContext<T extends NonNullable<unknown>>(): [
  () => T,
  React.Provider<T>,
] {
  const context = React.createContext<T>(null!);

  function useContext() {
    const c = React.useContext(context);
    if (c == null) {
      throw Error(
        'No value provided in deferred context, ensure that a value is provided before any consumer uses the context',
      );
    }
    return c;
  }

  return [useContext, context.Provider];
}
