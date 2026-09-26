"""Static layout smoke test for the offline explorer HTML."""
from html.parser import HTMLParser
from pathlib import Path


class Audit(HTMLParser):
    def __init__(self):
        super().__init__()
        self.options = 0
        self.has_viewport = False
        self.has_select = False
        self.has_chart = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'option': self.options += 1
        if tag == 'meta' and attrs.get('name') == 'viewport': self.has_viewport = True
        if tag == 'select' and attrs.get('id') == 'bag-select': self.has_select = True
        if tag == 'div' and attrs.get('id') == 'route_plot': self.has_chart = True


def main():
    html = (Path(__file__).resolve().parent / 'map_explorer.html').read_text(encoding='utf-8')
    audit = Audit(); audit.feed(html)
    assert audit.has_viewport and audit.has_select and audit.has_chart
    assert audit.options == 86, audit.options
    assert 'max-width:100%' in html and 'overflow-x:hidden' in html
    assert 'width:100%' in html and 'responsive": true' in html
    assert 'width=1300' not in html and 'width:1300' not in html
    assert "Plotly.restyle('route_plot'" in html
    print('Static HTML layout checks passed:',audit.options,'selectable runs, responsive chart, no fixed 1300 px width')


if __name__ == '__main__':main()
