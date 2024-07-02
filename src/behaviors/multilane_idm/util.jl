"""
    log1pxpexpy(x, y)

Compute `log(1 + x + exp(y))` in a numerically stable way for ``x \\geq 0``.
"""
function log1pxpexpy(x, y)
    log1px = log1p(x)
    return log1px + log1pexp(y - log1px)
end
