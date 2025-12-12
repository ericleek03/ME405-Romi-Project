document.addEventListener("DOMContentLoaded", () => {
  const content = document.querySelector(".markdown-body");
  const headers = content.querySelectorAll("h2, h3");

  if (!headers.length) return;

  const toc = document.createElement("nav");
  toc.className = "toc";
  toc.innerHTML = "<strong>Contents</strong>";

  headers.forEach(h => {
    const id = h.textContent
      .toLowerCase()
      .replace(/[^\w]+/g, "-");

    h.id = id;

    const link = document.createElement("a");
    link.href = `#${id}`;
    link.textContent = h.textContent;
    link.style.marginLeft = h.tagName === "H3" ? "1rem" : "0";

    toc.appendChild(link);
  });

  document.body.appendChild(toc);
});
