/** Only this local application origin may start a browser-controlled MCU.
 * CLI clients may omit Origin, but must still target the bound loopback host.
 */
export function allowLiveAccess(host: string | undefined, origin: string | undefined, port: number): boolean {
  if (host !== `127.0.0.1:${port}` && host !== `localhost:${port}`) return false;
  return origin === undefined || origin === `http://${host}`;
}
