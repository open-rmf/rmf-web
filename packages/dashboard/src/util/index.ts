export function toBlobUrl(data: Uint8Array) {
  const blob = new Blob([data]);
  return URL.createObjectURL(blob);
}

export type PromiseType<T> = T extends PromiseLike<infer U> ? U : never;