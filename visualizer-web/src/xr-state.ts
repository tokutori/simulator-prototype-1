export type XrUnavailableReason =
  | "insecure-context"
  | "api-unavailable"
  | "immersive-vr-unsupported"
  | "permission-denied";

export type XrState =
  | { readonly tag: "checking" }
  | { readonly tag: "unavailable"; readonly reason: XrUnavailableReason }
  | { readonly tag: "ready" }
  | { readonly tag: "presenting" };

export type XrMsg =
  | { readonly type: "availability"; readonly supported: true }
  | {
      readonly type: "availability";
      readonly supported: false;
      readonly reason: XrUnavailableReason;
    }
  | { readonly type: "session-started" }
  | { readonly type: "session-ended" };

export interface XrPresentation {
  readonly label: string;
  readonly detail: string;
  readonly presenting: boolean;
}

export function updateXr(state: XrState, message: XrMsg): XrState {
  switch (message.type) {
    case "availability":
      if (state.tag === "presenting") return state;
      return message.supported
        ? { tag: "ready" }
        : { tag: "unavailable", reason: message.reason };
    case "session-started":
      return { tag: "presenting" };
    case "session-ended":
      return state.tag === "presenting" ? { tag: "ready" } : state;
  }
}

export function presentXr(state: XrState): XrPresentation {
  switch (state.tag) {
    case "checking":
      return { label: "VR CHECKING", detail: "Checking immersive VR support", presenting: false };
    case "ready":
      return { label: "VR READY", detail: "Seated cockpit VR is available", presenting: false };
    case "presenting":
      return { label: "VR ACTIVE", detail: "Head-tracked cockpit view is active", presenting: true };
    case "unavailable":
      return unavailablePresentation(state.reason);
  }
}

function unavailablePresentation(reason: XrUnavailableReason): XrPresentation {
  switch (reason) {
    case "insecure-context":
      return {
        label: "VR NEEDS HTTPS",
        detail: "WebXR requires HTTPS, except on the same device through localhost",
        presenting: false,
      };
    case "api-unavailable":
      return {
        label: "VR UNAVAILABLE",
        detail: "This browser or device does not expose WebXR",
        presenting: false,
      };
    case "immersive-vr-unsupported":
      return {
        label: "VR UNSUPPORTED",
        detail: "This browser and headset combination does not support immersive VR",
        presenting: false,
      };
    case "permission-denied":
      return {
        label: "VR NOT ALLOWED",
        detail: "The browser denied the WebXR capability check",
        presenting: false,
      };
  }
}
